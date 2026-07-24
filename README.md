手动绑定更新


## 更新日期
2026-07-24

## 更新概述

将摄像头绑定方式从**自动扫描绑定**改为**手动绑定**，解决内置摄像头干扰、USB总线冲突、OSD模板误匹配等问题。

---

## 修改文件列表

| 文件 | 修改内容 |
|------|----------|
| `server_app/app.py` | 启动时不再自动扫描绑定，仅初始化 binder/estimator，等待用户手动绑定 |
| `server_app/vision.py` | `bind_vision_until_ready()` 不再循环扫描，仅写入状态后返回 |
| `server_app/routes/vision.py` | 新增手动绑定API、列出摄像头API；移除MJPEG流中的自动扫描回退；scan API增加CameraManager互斥保护 |
| `Car_vision_system/car_vision_system.py` | `list_available_cameras()` 增加缩略图和OSD频道匹配；`_match_channel()` 增加Top3分数日志和差距警告；`scan_and_bind()` 增加冲突检测；删除死代码 |
| `templates/index.html` | 扫描摄像头后显示缩略图预览、OSD频道、建议绑定信息；下拉菜单自动预选建议摄像头 |

---

## 新增 API

### `GET /api/vision/cameras`
列出系统中可用摄像头，包含缩略图和OSD频道匹配结果。

```json
{
  "success": true,
  "cameras": [
    {
      "index": 0,
      "width": 640,
      "height": 480,
      "thumbnail": "base64缩略图...",
      "detected_channel": "A1",
      "detected_score": 0.85,
      "suggested_car": "CAR3"
    }
  ]
}
```

### `POST /api/vision/bind/manual`
手动绑定摄像头到小车。

```json
// 请求
{ "binding": { "CAR3": 1, "CAR4": 2 } }

// 响应
{ "success": true, "message": "手动绑定完成", "bound_cameras": { "CAR3": 1, "CAR4": 2 } }
```

---

## 使用流程

1. 服务器启动后，控制台提示"请在控制面板中手动绑定摄像头"
2. 打开 Web 控制面板 → 摄像头绑定 → 点击**扫描可用摄像头**
3. 查看每个摄像头的缩略图预览和建议绑定信息
4. 为每辆小车选择对应摄像头索引，点击**确认绑定**
5. 绑定后视觉循环和MJPEG流自动生效

---

## 修复的问题

| 问题 | 原因 | 修复方式 |
|------|------|----------|
| 绑定后摄像头黑屏/无法显示 | 多线程绕过CameraManager直接打开摄像头，USB总线冲突 | 绑定API统一通过CameraManager操作，MJPEG流移除自动扫描回退 |
| 3车显示4车画面 | OSD模板匹配误判，无诊断日志 | `_match_channel()` 增加Top3分数和差距警告 |
| 扫描到内置笔记本摄像头 | 自动扫描无法排除无关设备 | 改为手动绑定，用户自主选择摄像头索引 |
| `stop_camera()` 重复释放 | 主线程和抓帧线程都调用 `cap.release()` | 去掉主线程的 `cap.release()`，由抓帧线程统一释放 |
