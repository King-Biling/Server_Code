import cv2
import os
import tkinter as tk
from tkinter import simpledialog

INDEX = 1
OSD_ROI = [10, 50, 30, 60]  # [Y起始, Y结束, X起始, X结束]

os.makedirs("templates", exist_ok=True)

# 初始化一个隐藏的 tkinter 主窗口，专门用于弹出输入框
root = tk.Tk()
root.withdraw()

try:
    cap = cv2.VideoCapture(INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap.isOpened(): 
        print(f"❌ 无法打开摄像头 {INDEX}，请检查设备！")
        exit()

    print("📸 连续截图模式已启动！")
    print("👉 操作说明：在 OpenCV 窗口按 'S' 保存当前模板，按 'Q' 退出程序。")

    while True:
        ret, frame = cap.read()
        if not ret: 
            continue

        display_frame = frame.copy()
        cv2.rectangle(display_frame, (OSD_ROI[2], OSD_ROI[0]), (OSD_ROI[3], OSD_ROI[1]), (0, 0, 255), 2)
        cv2.imshow("1. Look at the RED BOX", display_frame)

        roi = frame[OSD_ROI[0]:OSD_ROI[1], OSD_ROI[2]:OSD_ROI[3]]
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        cv2.imshow("2. Grayscale ROI (Press S)", gray_roi)

        key = cv2.waitKey(1) & 0xFF
        
        # 按下 S 键触发保存
        if key == ord('s') or key == ord('S'):
            # 使用 GUI 弹窗获取频道名称，避免阻塞控制台
            save_name = simpledialog.askstring("保存模板", "请输入当前频道名称 (如 A1, B2):", parent=root)
            
            if save_name and save_name.strip():
                save_name = save_name.strip()
                file_path = f"templates/{save_name}.png"
                cv2.imwrite(file_path, gray_roi)
                print(f"✅ 保存成功: {file_path}")
            else:
                print("⚠️ 已取消保存。")

        # 按下 Q 键退出程序
        elif key == ord('q') or key == ord('Q'):
            print("👋 已退出程序。")
            break

finally:
    if 'cap' in locals() and cap.isOpened(): 
        cap.release()
    cv2.destroyAllWindows()
    root.destroy()