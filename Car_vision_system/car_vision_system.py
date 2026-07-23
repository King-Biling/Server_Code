# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
import time

# ================= 全局系统配置 =================
CONFIG = {
    "CHANNEL_MAP": {
        "A8": "CAR1", "E1": "CAR2", "A1": "CAR3", "R8": "CAR4",
        # "A5": "CAR5", "A6": "CAR6", "A7": "CAR7", "A8": "CAR8"
    },
    "TEMPLATE_DIR": "templates",
    "SEARCH_ROI": [0, 100, 0, 200],  # [Y起始, Y结束, X起始, X结束]
    "TAG_SIZE": 0.050,                # AprilTag 物理边长(米)
    "FRAME_WIDTH": 640,
    "FRAME_HEIGHT": 480,
    # 检测下采样倍率：在 1/DETECT_SCALE 分辨率上跑 detectMarkers，角点再放大回全分辨率并 subpix 精修。
    # 2 = 在 320x240 上检测(推荐，卡顿明显缓解且精度基本无损)；1 = 全分辨率检测(最慢)；3 = 更快但远距离小标签可能漏检。
    "DETECT_SCALE": 2
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
        print(f"📂 [Binder] 成功加载 {len(self.template_dict)} 个频道模板: {list(self.template_dict.keys())}")

    def _open_camera(self, index):
        """安全打开相机，强制 MJPG 与分辨率限制"""
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["FRAME_WIDTH"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["FRAME_HEIGHT"])
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            print(f"📷 [Binder] 摄像头索引 {index} 打开成功，分辨率 {actual_w}x{actual_h}")
        else:
            print(f"⚠️ [Binder] 摄像头索引 {index} 打开失败")
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
        print(" [Binder] 启动 OSD 频道扫描 (请保持图传蓝屏状态)...")

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
                print(f" 成功: 索引 [{index}] -> 频道 '{best_channel}' -> 绑定到小车 [{car_id}]")
            
            cap.release()
            time.sleep(0.1) # 保护 USB 总线

        print(f" [Binder] 绑定完成，当前映射关系: {bound_cameras}\n" + "="*50)
        return bound_cameras


class PoseEstimator:
    """3D 位姿解算模块：负责通过 AprilTag 检测与相机位姿计算"""
    
    def __init__(self, config):
        self.config = config
        self.camera_params_cache = {} # 缓存小车相机参数

        # 初始化 AprilTag 检测器
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        aruco_params = cv2.aruco.DetectorParameters()

        # === 性能关键调参：抑制“画面出现标签后候选四边形爆炸导致解码耗时飙升”的卡顿 ===
        # 现象根因：detectMarkers 在高对比标签入画后会产生大量候选四边形，
        # 每个候选都要用 36h11 字典(587码×4旋转)做汉明匹配，检测耗时从几毫秒涨到几十毫秒，
        # 检测循环被拖慢 → 画面卡顿 + 帧陈旧 → 反而“识别不到”。以下参数用于砍掉无谓候选、
        # 加快字典识别的早期拒绝，同时我们改在缩小图上检测（见 process_frame）。
        try:
            # 候选周长范围：过滤掉过小/过大的轮廓，直接减少候选数量
            aruco_params.minMarkerPerimeterRate = 0.03
            aruco_params.maxMarkerPerimeterRate = 1.0
            # 自适应阈值窗口：收窄范围+加大步长，减少阈值化 pass 数量
            aruco_params.adaptiveThreshWinSizeMin = 5
            aruco_params.adaptiveThreshWinSizeMax = 15
            aruco_params.adaptiveThreshWinSizeStep = 10
            # 字典识别的早期拒绝：降低纠错率、限制边框误码，快速丢弃非标签候选
            aruco_params.errorCorrectionRate = 0.4
            aruco_params.maxErroneousBitsInBorderRate = 0.2
            # 角点细化交给我们自己在全分辨率上做 subpix，这里关掉以省时
            aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        except Exception as e:
            print(f"[Estimator] 调整检测器参数失败(使用默认值): {e}")

        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # 检测下采样倍率：在 1/DETECT_SCALE 分辨率上做检测，之后把角点放大回全分辨率并用
        # cornerSubPix 精修，兼顾“检测提速(候选更少、每候选更便宜)”与“位姿精度不损失”。
        self.detect_scale = int(self.config.get("DETECT_SCALE", 2))
        if self.detect_scale < 1:
            self.detect_scale = 1
        self._subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

        # 最近一次检测到的全分辨率角点(shape (4,2))，供“显示线程”轻量叠加标签框用；
        # None 表示当前帧未检测到。检测线程写、MJPEG 线程读，仅用于显示允许极短竞态。
        self.last_tag_corners = None

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
            print(f"⚠️[Estimator] 找不到 {car_id} 的标定文件 {filename}")
            self.camera_params_cache[car_id] = (None, None)
            return None, None

        print(f"📌 [Estimator] 使用标定文件: {params_path}")
        data = np.load(params_path)
        self.camera_params_cache[car_id] = (data['mtx'], data['dist'])
        return data['mtx'], data['dist']

    def process_frame(self, frame, car_id):
        """
        处理单帧图像，解算位姿误差
        返回: (解算是否成功, z_dist, x_offset, yaw_angle, 绘制了结果的图像)
        """
        camera_matrix, dist_coeffs = self.load_params_for_car(car_id)
        if camera_matrix is None:
            return False, 0, 0, 0, frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 在缩小图上做检测（候选更少、每候选更便宜），显著降低“标签入画后”的检测耗时。
        scale = self.detect_scale
        if scale > 1:
            small = cv2.resize(gray, None, fx=1.0 / scale, fy=1.0 / scale,
                               interpolation=cv2.INTER_AREA)
        else:
            small = gray
        corners, ids, _ = self.detector.detectMarkers(small)

        if ids is not None and len(ids) > 0:
            # 把缩小图上的角点坐标放大回全分辨率
            if scale > 1:
                corners = [c * float(scale) for c in corners]

            tag_corners = corners[0][0].astype(np.float32)

            # 在全分辨率灰度图上对角点做亚像素精修，找回下采样损失的精度。
            # 注意 cornerSubPix 要求 (N,1,2) 连续 float32 数组，且必须使用其返回值
            # （Python 版对传入数组的 in-place 更新不可靠）。这里之前直接传 (4,2)
            # 导致标签入画后每帧抛异常 -> 检测结果永远发布不出去（“检测不到标签”）。
            try:
                pts = np.ascontiguousarray(tag_corners.reshape(-1, 1, 2), dtype=np.float32)
                refined = cv2.cornerSubPix(
                    gray, pts, (5, 5), (-1, -1), self._subpix_criteria
                )
                tag_corners = refined.reshape(-1, 2).astype(np.float32)
            except Exception:
                # 精修失败不致命：退回缩放后的角点，仍可解算位姿
                pass

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

                # 保存全分辨率角点，供显示线程轻量画框（不在此处画，避免检测线程做绘制/编码）
                self.last_tag_corners = tag_corners.copy()

                return True, z_dist, x_offset, yaw_angle, frame

        # 未检测到有效标签
        self.last_tag_corners = None
        return False, 0, 0, 0, frame


# ================= 模拟服务器主程序 =================
def main():
    # 1. 实例化模块
    binder = DeviceBinder(CONFIG)
    estimator = PoseEstimator(CONFIG)
    
    # 2. 执行硬件绑定
    bound_cameras = binder.scan_and_bind()
    if not bound_cameras:
        print("❌未绑定任何设备，系统退出")
        return

    print("🚀系统已就绪，请【开启小车图传电源】！")
    print("👉按键 [1-8] 切换小车视角 | [Q] 退出")

    # 3. 初始化热切换状态
    active_car_id = list(bound_cameras.keys())[0]
    cap = None

    try:
        while True:
            # 维护相机流（按需打开，释放带宽）
            if cap is None:
                cam_idx = bound_cameras.get(active_car_id)
                if cam_idx is not None:
                    print(f"切换到小车 {active_car_id} (USB 索引: {cam_idx})")
                    cap = binder._open_camera(cam_idx) # 复用安全打开相机的逻辑
            
            ret, frame = cap.read() if cap else (False, None)
            
            if ret and frame is not None:
                success, z, x, yaw, out_frame = estimator.process_frame(frame, active_car_id)

                robot_suffix = active_car_id[3:] if active_car_id.upper().startswith("CAR") and len(active_car_id) > 3 else active_car_id
                robot_label = f"ROBOT-{robot_suffix}"
                pose_text = f"X:{z * 100.0:.1f}cm Y:{-x * 100.0:.1f}cm Yaw:{yaw:.1f}deg"
                label_font = cv2.FONT_HERSHEY_SIMPLEX
                label_scale = 0.7
                label_thickness = 2
                pose_scale = 0.65
                pose_thickness = 2
                label_size = cv2.getTextSize(robot_label, label_font, label_scale, label_thickness)[0]
                pose_size = cv2.getTextSize(pose_text, label_font, pose_scale, pose_thickness)[0]
                cv2.rectangle(out_frame, (10, 10), (10 + label_size[0] + 18, 10 + label_size[1] + 18), (0, 0, 0), -1)
                cv2.putText(out_frame, robot_label, (18, 10 + label_size[1] + 6), label_font, label_scale, (0, 165, 255), label_thickness)
                pose_bottom_y = max(20 + pose_size[1], out_frame.shape[0] - 12)
                pose_top_y = max(0, pose_bottom_y - pose_size[1] - 14)
                cv2.rectangle(out_frame, (10, pose_top_y), (10 + pose_size[0] + 18, min(out_frame.shape[0], pose_bottom_y + 8)), (0, 0, 0), -1)
                cv2.putText(out_frame, pose_text, (18, pose_bottom_y), label_font, pose_scale, (255, 255, 255), pose_thickness)
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
    