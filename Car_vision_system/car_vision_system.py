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
    "DETECT_SCALE": 2,
    "SKIP_CAMERA_INDICES": [],#修改这个列表即可控制跳过哪些摄像头索引，如 [0, 1] 跳过索引 0 和 1，[] 不跳过任何摄像头。
}
# ==========================================

class DeviceBinder:
    """硬件设备智能绑定模块：负责通过 OSD 水印识别相机对应的物理小车"""
    
    def __init__(self, config):
        self.config = config
        self.template_dict = {}
        self._load_templates()

    def _load_templates(self):
        """加载灰度 OSD 模板到内存"""
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
        print(f" [Binder] 成功加载 {len(self.template_dict)} 个频道模板: {list(self.template_dict.keys())}")

    def _open_camera(self, index, verbose=True):
        """安全打开相机，强制 MJPG 与分辨率限制"""
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["FRAME_WIDTH"])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["FRAME_HEIGHT"])
            if verbose:
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                print(f" [Binder] 摄像头索引 {index} 打开成功，分辨率 {actual_w}x{actual_h}")
        else:
            if verbose:
                print(f" [Binder] 摄像头索引 {index} 打开失败")
        return cap

    def _match_channel(self, frame):
        """纯灰度大范围滑窗匹配 OSD 频道，返回 (频道名, 分数)，并打印 Top3 候选"""
        roi = frame[self.config["SEARCH_ROI"][0]:self.config["SEARCH_ROI"][1], 
                    self.config["SEARCH_ROI"][2]:self.config["SEARCH_ROI"][3]]
        gray_search_area = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        best_match = None
        highest_score = 0.0
        all_scores = {}

        for channel_name, template in self.template_dict.items():
            if gray_search_area.shape[0] < template.shape[0] or gray_search_area.shape[1] < template.shape[1]:
                continue
            res = cv2.matchTemplate(gray_search_area, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(res)
            all_scores[channel_name] = max_val
            
            if max_val > highest_score and max_val > 0.70:
                highest_score = max_val
                best_match = channel_name

        if best_match:
            sorted_scores = sorted(all_scores.items(), key=lambda x: x[1], reverse=True)[:3]
            top3_str = ", ".join(f"{ch}={sc:.3f}" for ch, sc in sorted_scores)
            gap = sorted_scores[0][1] - (sorted_scores[1][1] if len(sorted_scores) > 1 else 0)
            print(f"   [匹配] 最佳={best_match}({highest_score:.3f}) 差距={gap:.3f} Top3: {top3_str}")
            if gap < 0.05:
                print(f"   [匹配警告] 最佳与次佳分数差距过小({gap:.3f})，可能误匹配！")

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
                
            for _ in range(10): cap.read()
            
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
                if car_id in bound_cameras:
                    old_index = bound_cameras[car_id]
                    print(f"   [冲突] 小车 [{car_id}] 已绑定到索引 [{old_index}]，新索引 [{index}] 跳过(保留先匹配的结果)")
                else:
                    bound_cameras[car_id] = index
                    print(f" 成功: 索引 [{index}] -> 频道 '{best_channel}' -> 绑定到小车 [{car_id}]")
            else:
                if best_channel:
                    print(f"   跳过: 索引 [{index}] 匹配到频道 '{best_channel}' 但不在 CHANNEL_MAP 中")
            
            cap.release()
            time.sleep(0.1)

        used_indices = list(bound_cameras.values())
        duplicates = [i for i in set(used_indices) if used_indices.count(i) > 1]
        if duplicates:
            print(f"   [绑定异常] 以下摄像头索引被多辆车使用: {duplicates}，绑定结果不可靠！")

        print(f" [Binder] 绑定完成，当前映射关系: {bound_cameras}\n" + "="*50)
        return bound_cameras

    def list_available_cameras(self, max_cameras=6):
        """枚举系统中可用的摄像头索引，返回每路摄像头的索引和分辨率。跳过 SKIP_CAMERA_INDICES 中的索引。"""
        skip_indices = set(self.config.get("SKIP_CAMERA_INDICES", []))
        available = []
        for index in range(max_cameras):
            if index in skip_indices:
                continue
            cap = self._open_camera(index, verbose=False)
            if cap.isOpened():
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
                available.append({
                    "index": index,
                    "width": w,
                    "height": h,
                })
                cap.release()
            else:
                cap.release()
        if available:
            print(f" [Binder] 发现 {len(available)} 个可用摄像头: {[c['index'] for c in available]}")
        else:
            print(f" [Binder] 未发现可用摄像头")
        return available

    def manual_bind(self, binding_map):
        """手动绑定摄像头到小车。binding_map: {CAR_ID: CAMERA_INDEX}，如 {"CAR1": 0, "CAR2": 2}"""
        bound_cameras = {}
        for car_id, cam_index in binding_map.items():
            cap = self._open_camera(cam_index)
            if cap.isOpened():
                bound_cameras[car_id] = cam_index
                cap.release()
                print(f" [Binder] 手动绑定: 小车 [{car_id}] -> 摄像头索引 [{cam_index}]")
            else:
                cap.release()
                print(f" [Binder] 手动绑定失败: 摄像头索引 [{cam_index}] 无法打开，小车 [{car_id}] 跳过")
        print(f" [Binder] 手动绑定完成，映射关系: {bound_cameras}")
        return bound_cameras


class PoseEstimator:
    """3D 位姿解算模块：负责通过 AprilTag 检测与相机位姿计算"""
    
    def __init__(self, config):
        self.config = config
        self.camera_params_cache = {}

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        aruco_params = cv2.aruco.DetectorParameters()

        try:
            aruco_params.minMarkerPerimeterRate = 0.03
            aruco_params.maxMarkerPerimeterRate = 1.0
            aruco_params.adaptiveThreshWinSizeMin = 5
            aruco_params.adaptiveThreshWinSizeMax = 15
            aruco_params.adaptiveThreshWinSizeStep = 10
            aruco_params.errorCorrectionRate = 0.4
            aruco_params.maxErroneousBitsInBorderRate = 0.2
            aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        except Exception as e:
            print(f"[Estimator] 调整检测器参数失败(使用默认值): {e}")

        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.detect_scale = int(self.config.get("DETECT_SCALE", 2))
        if self.detect_scale < 1:
            self.detect_scale = 1
        self._subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

        self.last_tag_corners = None

        half_s = self.config["TAG_SIZE"] / 2.0
        self.object_points = np.array([
            [-half_s,  half_s, 0], [ half_s,  half_s, 0],
            [ half_s, -half_s, 0], [-half_s, -half_s, 0]
        ], dtype=np.float32)

        # ROI 粗定位器（移植自 QRCodeReader 的轮廓树检测，失败时回退整图）
        from tag_roi_locator import TagRoiLocator
        roi_cfg = self.config.get("ROI_LOCATOR", {})
        self.roi_locator = TagRoiLocator(
            min_level=roi_cfg.get("min_level", 2),
            area_ratio_min=roi_cfg.get("area_ratio_min", 1.5),
            area_ratio_max=roi_cfg.get("area_ratio_max", 8.0),
            pad_ratio=roi_cfg.get("pad_ratio", 0.2),
            min_contour_area=roi_cfg.get("min_contour_area", 100),
        )

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
            print(f"[Estimator] 找不到 {car_id} 的标定文件 {filename}")
            self.camera_params_cache[car_id] = (None, None)
            return None, None

        print(f"[Estimator] 使用标定文件: {params_path}")
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

        # === ROI 粗定位（移植自 QRCodeReader 的轮廓树检测，失败回退整图）===
        rx, ry, rw, rh = self.roi_locator.estimate_roi(gray)
        roi_gray = gray[ry:ry + rh, rx:rx + rw]

        scale = self.detect_scale
        if scale > 1:
            small = cv2.resize(roi_gray, None, fx=1.0 / scale, fy=1.0 / scale,
                               interpolation=cv2.INTER_AREA)
        else:
            small = roi_gray
        corners, ids, _ = self.detector.detectMarkers(small)

        if ids is not None and len(ids) > 0:
            if scale > 1:
                corners = [c * float(scale) for c in corners]
            # 补偿 ROI 偏移，把角点还原到全图坐标系（solvePnP 仍用原内参）
            corners = [c + np.array([rx, ry], dtype=np.float32) for c in corners]

            tag_corners = corners[0][0].astype(np.float32)

            try:
                pts = np.ascontiguousarray(tag_corners.reshape(-1, 1, 2), dtype=np.float32)
                refined = cv2.cornerSubPix(
                    gray, pts, (5, 5), (-1, -1), self._subpix_criteria
                )
                tag_corners = refined.reshape(-1, 2).astype(np.float32)
            except Exception:
                pass

            success, rvec, tvec = cv2.solvePnP(
                self.object_points, tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                x_offset = tvec[0][0]
                z_dist = tvec[2][0]
                rmat, _ = cv2.Rodrigues(rvec)
                euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                yaw_angle = euler_angles[1]

                self.last_tag_corners = tag_corners.copy()

                return True, z_dist, x_offset, yaw_angle, frame

        self.last_tag_corners = None
        return False, 0, 0, 0, frame


# ================= 模拟服务器主程序 =================
def main():
    binder = DeviceBinder(CONFIG)
    estimator = PoseEstimator(CONFIG)
    
    bound_cameras = binder.scan_and_bind()
    if not bound_cameras:
        print("未绑定任何设备，系统退出")
        return

    print("系统已就绪，请开启小车图传电源！")
    print("按键 [1-8] 切换小车视角 | [Q] 退出")

    active_car_id = list(bound_cameras.keys())[0]
    cap = None

    try:
        while True:
            if cap is None:
                cam_idx = bound_cameras.get(active_car_id)
                if cam_idx is not None:
                    print(f"切换到小车 {active_car_id} (USB 索引: {cam_idx})")
                    cap = binder._open_camera(cam_idx)
            
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
