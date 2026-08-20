# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: tag_roi_locator
 模块职责: 基于 QRCodeReader 的轮廓树嵌套层级 + 面积比方法，粗定位 AprilTag 的 ROI
--------------------------------------------------------------------------------
 本模块移植自 https://github.com/Griffintaur/QRCodeReader 的：
   Imagehandler.GetImageContour        (自适应阈值 + Canny + findContours)
   PatternFinding.IsPossibleQRContour  (轮廓树嵌套层级判定)
   PatternFinding.CheckingRatioOfContours (相邻层面积比判定)

 原版针对 QR Finder Pattern (3 个嵌套黑白环, level>=6)，这里改造为针对
 AprilTag 36h11 (1 个外黑边-白边-数据区, level>=2)，并取面积最大的候选作为 ROI。

 失败时返回整图 ROI，不阻断上游 AprilTag 检测流程（回退兜底）。
================================================================================
"""

import cv2
import numpy as np


class TagRoiLocator:
    """用轮廓树嵌套层级 + 面积比粗定位 AprilTag 的外框 ROI。"""

    def __init__(self, min_level=2, area_ratio_min=1.5, area_ratio_max=8.0,
                 pad_ratio=0.2, min_contour_area=100):
        """
        Args:
            min_level:        轮廓树最小嵌套层数(AprilTag 约 2~3, QR 为 6)
            area_ratio_min:   相邻层面积比下限
            area_ratio_max:   相邻层面积比上限
            pad_ratio:        ROI 外扩余量比例
            min_contour_area: 过滤太小的噪点轮廓
        """
        self.min_level = min_level
        self.area_ratio_min = area_ratio_min
        self.area_ratio_max = area_ratio_max
        self.pad_ratio = pad_ratio
        self.min_contour_area = min_contour_area

    def _preprocess(self, gray):
        """自适应阈值 -> Canny -> findContours(RETR_TREE)。

        移植自 Imagehandler.__convertImagetoBlackWhite + GetImageContour。
        """
        bw = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
            11, 2
        )
        edges = cv2.Canny(bw, 100, 200)
        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        return contours, hierarchy

    def _nest_level(self, idx, hierarchy):
        """沿 hierarchy[0][idx][2] (first child) 逐层向内走，返回嵌套层数。

        移植自 PatternFinding.IsPossibleQRContour 的层数统计部分。
        """
        child = hierarchy[0][idx][2]
        level = 0
        while child != -1:
            level += 1
            child = hierarchy[0][child][2]
        return level

    def _area_ratio_ok(self, idx, contours, hierarchy):
        """相邻两层面积比判定。

        移植自 PatternFinding.CheckingRatioOfContours，放宽比值范围适配 AprilTag。
        AprilTag 边框宽度约标签尺寸的 1/8，理论面积比约 3.5，这里放宽到 [1.5, 8.0]。
        """
        c0 = hierarchy[0][idx][2]
        if c0 == -1:
            return False
        c1 = hierarchy[0][c0][2]
        if c1 == -1:
            return False
        a0 = cv2.contourArea(contours[idx])
        a1 = cv2.contourArea(contours[c0])
        a2 = cv2.contourArea(contours[c1])
        if a1 < 1.0 or a2 < 1.0:
            return False
        ratio = (a0 / a1) / (a1 / a2)
        return self.area_ratio_min < ratio < self.area_ratio_max

    def estimate_roi(self, gray):
        """粗定位 AprilTag 的外框 ROI。

        Args:
            gray: 灰度图 (H x W, uint8)

        Returns:
            (x, y, w, h) 像素坐标的 ROI 矩形。
            检测失败时返回整图 ROI (0, 0, W, H)，不阻断上游流程。
        """
        h, w = gray.shape[:2]

        try:
            contours, hierarchy = self._preprocess(gray)
        except Exception:
            return (0, 0, w, h)

        if hierarchy is None or len(contours) == 0:
            return (0, 0, w, h)

        candidates = []
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) < self.min_contour_area:
                continue
            if self._nest_level(i, hierarchy) >= self.min_level \
               and self._area_ratio_ok(i, contours, hierarchy):
                area = cv2.contourArea(contours[i])
                candidates.append((area, i))

        if not candidates:
            return (0, 0, w, h)

        # 取面积最大的候选(对应 AprilTag 外框)
        candidates.sort(key=lambda x: x[0], reverse=True)
        _, best = candidates[0]
        x, y, ww, hh = cv2.boundingRect(contours[best])

        # 外扩余量，确保标签角点在 ROI 内
        pad_x = int(ww * self.pad_ratio)
        pad_y = int(hh * self.pad_ratio)
        x = max(0, x - pad_x)
        y = max(0, y - pad_y)
        ww = min(w - x, ww + 2 * pad_x)
        hh = min(h - y, hh + 2 * pad_y)

        return (x, y, ww, hh)


if __name__ == '__main__':
    # 离线测试：读取一张实拍图，画 ROI 框
    import sys
    if len(sys.argv) < 2:
        print("用法: python tag_roi_locator.py <image_path>")
        sys.exit(1)
    img = cv2.imread(sys.argv[1])
    if img is None:
        print("无法读取图像:", sys.argv[1])
        sys.exit(1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    locator = TagRoiLocator()
    rx, ry, rw, rh = locator.estimate_roi(gray)
    print(f"ROI: x={rx} y={ry} w={rw} h={rh} (图像 {gray.shape[1]}x{gray.shape[0]})")
    cv2.rectangle(img, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
    cv2.imshow("Tag ROI", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
