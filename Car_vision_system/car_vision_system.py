# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
import time

# ================= 全局系统配置 =================
CONFIG = {
    "CHANNEL_MAP": {
        "A2": "CAR1", "A4": "CAR2", "A6": "CAR3", "A8": "CAR4",
        # "A5": "CAR5", "A6": "CAR6", "A7": "CAR7", "A8": "CAR8"
    },
    "TEMPLATE_DIR": "templates",
    "SEARCH_ROI": [0, 100, 0, 200],  # [Y起始, Y结束, X起始, X结束]
    "TAG_SIZE": 0.050,                # AprilTag 物理边长(米)
    "FRAME_WIDTH": 640,
    "FRAME_HEIGHT": 480
}
# ==========================================

class DeviceBinder:
    """硬件设备智能绑定模块：负责通过 OSD 水印识别相机对应的物理小车"""
    
    def __init__(self, config):
        self.config = config
        self.template_dict = {}
        self._load_templates()

    def _load_templates(self):
        """加载灰度 OSD 模板到内�?"""
        template_dir = self.config["TEMPLATE_DIR"]
        if not os.path.exists(template_dir):
            print(f" [Binder] 找不到模板文件夹 '{template_dir}'，请先运行截图脚本！")
            return
            
        for filename in os.listdir(template_dir):
            if filename.endswith(".png"):
                channel_name = filename.replace(".png", "")
                tmpl = cv2.imread(os.path.join(template_dir, filename), cv2.IMREAD_GRAYSCALE)
                if tmpl is not None:
                    self.template_dict[channel_name] = tmpl
        print(f"📂 [Binder] 成功加载 {len(self.template_dict)} 个频道模�?: {list(self.template_dict.keys())}")

    def _open_camera(self, index):
        """安全打开相机，强�? MJPG 与分辨率限制"""
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["FRAME_WIDTH"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["FRAME_HEIGHT"])
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            print(f"📷 [Binder] 摄像头索�? {index} 打开成功，分辨率 {actual_w}x{actual_h}")
        else:
            print(f"⚠️ [Binder] 摄像头索�? {index} 打开失败")
        return cap

    def _match_channel(self, frame):
        """纯灰度大范围滑窗匹配 OSD 频道"""
        roi = frame[self.config["SEARCH_ROI"][0]:self.config["SEARCH_ROI"][1], 
                    self.config["SEARCH_ROI"][2]:self.config["SEARCH_ROI"][3]]
        gray_search_area = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        best_match = None
        highest_score = 0.0

        for channel_name, template in self.template_dict.items():
            if gray_search_area.shape[0] < template.shape[0] or gray_search_area.shape[1] < template.shape[1]:
                continue
            res = cv2.matchTemplate(gray_search_area, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)
            
            if max_val > highest_score and max_val > 0.70:
                highest_score = max_val
                best_match = channel_name
                
        return best_match, highest_score

    def scan_and_bind(self, max_cameras=6):
        """执行开机盲扫，返回 {CAR_ID: CAMERA_INDEX} 映射字典"""
        bound_cameras = {}
        if not self.template_dict:
            return bound_cameras

        print("\n" + "="*50)
        print(" [Binder] 启动 OSD 频道扫描 (请保持图传蓝屏状�?)...")
        
        for index in range(max_cameras):
            cap = self._open_camera(index)
            if not cap.isOpened():
                cap.release()
                time.sleep(0.05)
                cap = self._open_camera(index)
                if not cap.isOpened():
                    cap.release()
                    continue
                
            for _ in range(10): cap.read() # 等待曝光稳定
            
            best_channel = None
            best_score = 0.0
            for _ in range(3):
                ret, frame = cap.read()
                if not ret:
                    continue
                channel, score = self._match_channel(frame)
                if channel and score > best_score:
                    best_score = score
                    best_channel = channel
                time.sleep(0.03)

            if best_channel and best_channel in self.config["CHANNEL_MAP"]:
                car_id = self.config["CHANNEL_MAP"][best_channel]
                bound_cameras[car_id] = index
                print(f" 成功: 索引 [{index}] -> 频道 '{best_channel}' -> 绑定到小�? [{car_id}]")
            
            cap.release()
            time.sleep(0.1) # 保护 USB 总线

        print(f" [Binder] 绑定完成，当前映射关�?: {bound_cameras}\n" + "="*50)
        return bound_cameras


class PoseEstimator:
    """3D 位姿解算模块：负责通过 AprilTag 检测与相机位姿计算"""
    
    def __init__(self, config):
        self.config = config
        self.camera_params_cache = {} # 缓存小车相机参数
        
        # 初始�?? AprilTag 检测器
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        
        # 定义 3D 物理角点
        half_s = self.config["TAG_SIZE"] / 2.0
        self.object_points = np.array([
            [-half_s,  half_s, 0], [ half_s,  half_s, 0],
            [ half_s, -half_s, 0], [-half_s, -half_s, 0]
        ], dtype=np.float32)

    def load_params_for_car(self, car_id):
        """加载并缓存特定小车的相机内参"""
        if car_id in self.camera_params_cache:
            return self.camera_params_cache[car_id]

        filename = f"camera_params_{car_id}.npz"
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
        root_path = os.path.join(project_root, filename)
        local_path = os.path.join(os.path.dirname(__file__), filename)

        params_path = None
        if os.path.exists(root_path):
            params_path = root_path
        elif os.path.exists(local_path):
            params_path = local_path

        if not params_path:
            print(f"⚠️[Estimator] 找不�? {car_id} 的标定文�? {filename}")
            self.camera_params_cache[car_id] = (None, None)
            return None, None

        print(f"📌 [Estimator] 使用标定文件: {params_path}")
        data = np.load(params_path)
        self.camera_params_cache[car_id] = (data['mtx'], data['dist'])
        return data['mtx'], data['dist']

    def process_frame(self, frame, car_id):
        """
        处理单帧图像，解算位姿误�?
        返回: (解算是否成功, z_dist, x_offset, yaw_angle, 绘制了结果的图像)
        """
        camera_matrix, dist_coeffs = self.load_params_for_car(car_id)
        if camera_matrix is None:
            return False, 0, 0, 0, frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            tag_corners = corners[0][0]
            tag_id = ids[0][0]
            
            # �?? 2D 边框
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            success, rvec, tvec = cv2.solvePnP(
                self.object_points, tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # 提取数据
                x_offset = tvec[0][0]
                z_dist = tvec[2][0]
                rmat, _ = cv2.Rodrigues(rvec)
                euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                yaw_angle = euler_angles[1]

                # 绘制 3D �??
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                return True, z_dist, x_offset, yaw_angle, frame

        return False, 0, 0, 0, frame


# ================= 模拟服务器主程序 =================
def main():
    # 1. 实例化模�??
    binder = DeviceBinder(CONFIG)
    estimator = PoseEstimator(CONFIG)
    
    # 2. 执行硬件绑定
    bound_cameras = binder.scan_and_bind()
    if not bound_cameras:
        print("❌未绑定任何设备，系统退�??")
        return

    print("🚀系统已就绪，请【开启小车图传电源】！")
    print("👉按键 [1-8] 切换小车视角 | [Q] 退�??")

    # 3. 初始化热切换状�?
    active_car_id = list(bound_cameras.keys())[0]
    cap = None

    try:
        while True:
            # 维护相机流（按需打开，释放带宽）
            if cap is None:
                cam_idx = bound_cameras.get(active_car_id)
                if cam_idx is not None:
                    print(f"切换�? {active_car_id} (USB 索引: {cam_idx})")
                    cap = binder._open_camera(cam_idx) # 复用安全打开相机的逻辑
            
            ret, frame = cap.read() if cap else (False, None)
            
            if ret and frame is not None:
                success, z, x, yaw, out_frame = estimator.process_frame(frame, active_car_id)
                
               
                # if success:
                #     send_command_to_car(active_car_id, z, x, yaw)

                cv2.putText(out_frame, f"VIEW: {active_car_id}", (15, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                cv2.imshow("Modular Vision System", out_frame)

            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif ord('1') <= key <= ord('8'):
                target_car = f"CAR{chr(key)}"
                if target_car in bound_cameras and target_car != active_car_id:
                    if cap:
                        cap.release() 
                        cap = None
                    active_car_id = target_car

    except KeyboardInterrupt:
        pass
    finally:
        if cap: cap.release()
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    main()