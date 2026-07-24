# 多车协同服务器端工程说明

本工程是多车协同实验平台的服务器端（上位机），负责统一调度多辆小车完成
遥测采集、编队控制、重构拼接/分离、末端视觉制导与实时监控。服务器同时提供
一个基于 Flask 的 Web 控制台，供操作者下发指令与观察运行状态。

本文档面向协同开发的三位成员，说明工程架构、模块职责、信息流走向以及各类
通信协议，作为共同遵循的接口契约。

---

## 一、总体架构

系统由三类角色构成：

- 上位机服务器（本工程）：调度者。运行 UDP 服务器与 Flask Web 服务，
  并在本地跑视觉检测线程。
- 小车（下位机）：执行者。上报遥测、执行运动/编队/重构指令，在重构拼接阶段
  上报图传画面。
- Web 前端（浏览器）：操作界面。通过 HTTP 与服务器交互，通过 MJPEG 观看视频流。

```
        +---------------------+           HTTP / MJPEG            +----------------+
        |    Web 前端(浏览器)  | <------------------------------> |                |
        +---------------------+                                   |                |
                                                                  |   上位机服务器  |
        +---------------------+     UDP 遥测(上行 8080)           |   (本工程)     |
        |                     | -------------------------------> |                |
        |     小车 x N        |                                   |   Flask :5000  |
        |  (下位机执行者)      | <------------------------------- |   UDP   :8080  |
        |                     |     UDP 子网广播/指令(下行 8081)   |   广播  :8081  |
        +---------------------+                                   +----------------+
              |   ^                                                      ^
              |   | USB 图传(模拟视频)                                    | OpenCV 采集
              +---+------------------------------------------------------+
                        采集卡/摄像头(服务器本地, 视觉检测)
```

要点：

- 遥测上行走 UDP 8080（小车 -> 服务器）。
- 指令下行主要走 UDP 子网广播 8081（服务器 -> 全体小车，小车按编号自行过滤）。
  少量低延迟遥控走单播。
- 小车图传通过采集卡接入服务器本地，由 OpenCV 抓帧，服务器端跑 AprilTag
  检测与位姿解算，再据此生成末端制导速度并广播下发。
- Web 前端不直接与小车通信，一切经由服务器中转。

---

## 二、目录结构与模块职责

重构后，原单体文件 `web_car_server.py`（约 3000 行）已按功能域拆分到
`server_app/` 子包中。工程根目录的 `web_car_server.py` 现为薄封装启动入口。

```
Server_Web_UDP_26_7_14/
├── web_car_server.py              启动入口(薄封装, 转调 server_app.app.main)
├── web_car_server_legacy_backup.py 重构前的历史单体文件(仅供对照, 不参与运行)
├── formation_controller.py        编队控制蓝图(领航-跟随, 矩形轨迹实验)
├── requirements.txt               依赖清单
├── templates/index.html           Web 控制台前端页面
├── Car_vision_system/             视觉引擎(设备绑定 + AprilTag 位姿解算)
│   ├── car_vision_system.py       DeviceBinder / PoseEstimator / CONFIG
│   └── capture_template.py        OSD 频道模板截图工具
├── logs/                          运行日志与自动归档
├── pose_data/                     位姿采样 CSV 落盘目录
└── server_app/                    服务器端应用主包
    ├── __init__.py                包说明与模块清单
    ├── config.py                  全局常量与运行时可调参数
    ├── state.py                   跨模块共享的可变状态与线程锁
    ├── logging_utils.py           重构事件持久化日志与自动归档
    ├── models.py                  纯数据模型(Car / PoseSaveSession)
    ├── net.py                     UDP 服务器 / 广播服务器 / 网络辅助
    ├── vision.py                  摄像头管理 / 视觉主循环 / 画面叠加 / 设备绑定
    ├── guide.py                   末端制导控制器与航向误差计算
    ├── topology.py                通信拓扑矩阵构建 / 下发 / 缓存
    ├── reconstruct.py             重构(拼接/分离)状态机与报文处理
    ├── pose_save.py               位姿数据定时采样与落盘
    ├── app.py                     Flask 应用装配与程序主入口
    └── routes/                    Flask 蓝图(按业务域拆分的 HTTP 接口)
        ├── __init__.py            蓝图统一注册
        ├── page.py                页面入口 /
        ├── cars.py                小车列表 /api/cars
        ├── broadcast.py           广播开关与参数 /api/broadcast*
        ├── control.py             单车位置/速度控制 /api/control_*
        ├── topology.py            通信拓扑 /api/topology*
        ├── reconstruct.py         重构流程 /api/reconstruct*
        ├── vision.py              视觉监控与视频流 /api/vision*, /stream/mjpeg
        └── pose.py                位姿保存 /api/save_pose
```

### 模块依赖分层

从底层到高层，依赖方向单向向下（高层依赖低层）：

```
第0层(无依赖)     : config
第1层             : state  (依赖 config)
第2层             : models, logging_utils  (依赖 config/state)
第3层             : topology, pose_save     (依赖 config/state/models)
第4层             : net, vision, guide      (依赖上述各层)
第5层(业务核心)   : reconstruct             (依赖 config/state/logging_utils/topology/guide)
第6层(接口)       : routes/*                (依赖各业务模块)
第7层(装配)       : app                     (装配全部)
```

net 层与 reconstruct 层、vision 层与 reconstruct 层之间存在双向调用
（收发报文 <-> 驱动状态机；视觉检测 <-> 更新制导）。为避免循环导入，
这些跨层调用一律采用两种方式之一：

1. 通过 `state.udp_server` 运行时单例访问网络能力；
2. 在函数内部使用延迟导入（import inside function）。

---

## 三、关键设计约定（协同开发必读）

### 1. 共享可变状态集中管理

所有会被运行时重新赋值、且需跨模块共享的量，统一存放于 `config.py` 与
`state.py`。其它模块必须以 `模块名.变量` 的形式读写，例如：

```python
from . import state
state.broadcast_enabled = True          # 正确：赋值可被其它模块看到
```

严禁使用 `from state import broadcast_enabled` 拷贝取值后再赋值，否则修改
无法跨模块传播，会破坏运行行为。

属于该类的量：
- `config`: `PREP_RETRY_TIMEOUT`、`GUIDE_FUNNEL_Y_THRESHOLD` 等可调参数。
- `state`: `broadcast_enabled`、`broadcast_interval`、`broadcast_group_size`、
  `communication_topology`、`topology_enabled`、`topology_cache`、`udp_server`。

### 2. 线程与锁

服务器是重度多线程的。主要线程与其保护的状态：

| 线程 | 来源 | 职责 | 主要访问的共享状态 |
|------|------|------|------------------|
| UDP 接收 | net `_receive_loop` | 收遥测/重构报文并分派 | cars, reconstruct_state |
| 状态广播 | net `_broadcast_loop` | 周期广播小车状态 | cars, topology |
| 健康检查 | net `_health_check_loop` | 超时标记断开 | cars |
| 离线清理 | net `_cleanup_loop` | 清理长期离线小车 | cars |
| STEP 守护 | net `_step_watchdog_loop` | STEP/首图超时重试 | reconstruct_state |
| 视觉检测 | vision `vision_loop` | 抓帧检测解算, 更新制导 | vision_state, reconstruct_state |
| 摄像头抓帧 | vision CameraManager | 每相机独立抓帧缓存 | 相机内部状态 |
| 制导 | guide GuideController | 每车独立周期下发 GUIDE | reconstruct_state |
| 日志归档 | logging_utils | 定时压缩归档日志 | 日志文件 |

对应的锁：
- `state.car_lock`：保护 `cars` 表。
- `state.reconstruct_lock`（RLock，可重入）：保护 `reconstruct_state`。
- `state.vision_lock`：保护 `vision_state`。
- `state.save_lock`：保护位姿保存会话表。
- `state.file_log_lock`：串行化日志文件写入。

注意：`reconstruct_lock` 使用可重入锁，因为部分函数在已持锁时会调用同样需要
加锁的辅助函数（如 `enter_error_state`）。修改时请勿改为普通 Lock。

---

## 四、信息流

### 1. 遥测上行流（小车 -> 服务器）

```
小车周期发送 "CARn:x,y,yaw,voltage,vx,vy,vz"
   -> UDP 8080
   -> net._receive_loop 收字节
   -> net._handle_car_data 解析
   -> 更新 state.cars[CARn] (位置/朝向/电量/速度/连接状态)
   -> 记录心跳事件, 触发位姿采样(若在保存会话中)
   -> 新连接/重连时回发 RECONNECT_ACK 并立即广播一次
```

### 2. 状态广播下行流（服务器 -> 全体小车）

```
net._broadcast_loop 每 broadcast_interval 秒触发
   -> 收集 3 秒内活跃的已连接小车
   -> 若启用拓扑, 按矩阵过滤出允许广播的小车
   -> 按 broadcast_group_size 分组
   -> 组装 "[N C1 x y yaw vx vy vz C2 ...]" 广播报文
   -> BroadcastServer 经 UDP 子网广播 8081 下发
   -> 小车据此获知僚车状态(用于编队/重构相对定位)
```

### 3. 视觉与制导闭环（重构拼接阶段）

```
采集卡图传 -> CameraManager 抓帧缓存
   -> vision_loop 取当前待拼接车对应相机的帧
   -> PoseEstimator 解算 AprilTag 位姿, 得到 (前进/横向/航向)误差
   -> 写入 vision_state.overlay_* (供 MJPEG 叠加显示)
   -> update_guide_controller 把误差喂给该车的 GuideController
   -> GuideController 按 P 控制 + 漏斗解耦 + 限幅算出 (vx,vy,vz)
   -> 经子网广播下发 "[R,GUIDE,CARn,VX=..,VY=..,VZ=..,DONE=0|1]"
   -> 小车执行, 到位后回 STEP_OK, 服务器推进下一辆
```

### 4. Web 监控视频流

```
浏览器 <img src="/stream/mjpeg/CARn">
   -> routes/vision.mjpeg_stream 打开对应相机
   -> 直接取采集线程原始帧, 轻量叠加 overlay(标签框/编号/误差/速度)
   -> 编码为 JPEG, 以 multipart/x-mixed-replace 持续推送
```

显示与检测彻底解耦：检测线程只写“角点/误差/有无 Tag”数据，不做 JPEG 编码；
MJPEG 线程独立取帧、叠加、编码。这样检测耗时尖峰（标签入画）不拖累显示帧率。

---

## 五、通信协议

服务器与小车之间的 UDP 报文分为若干族，用首字段区分。除特别说明外均为
UTF-8 文本，形如 `[类别,...]`。

### 1. 遥测上行（小车 -> 服务器，UDP 8080）

普通遥测采用冒号分隔格式（非方括号）：

```
CARn:x,y,yaw,voltage,vx,vy,vz
```

| 字段 | 含义 | 单位 |
|------|------|------|
| x, y | 平面坐标 | m |
| yaw | 航向角 | deg |
| voltage | 电池电压 | V |
| vx, vy, vz | 线速度与角速度 | m/s, rad/s |

服务器回发的重连确认：`RECONNECT_ACK:CARn,SERVER_READY`

### 2. 状态广播（服务器 -> 小车，UDP 广播 8081）

```
[N C1 x y yaw vx vy vz C2 x y yaw vx vy vz ...]
```

- `N`：本组小车数量。
- `Ck`：小车短编号（`C` + 编号末位，如 CAR1 -> C1）。
- 数值精度：坐标 2 位、航向 1 位、速度 4 位小数。

### 3. 运动控制（服务器 -> 小车）

| 指令 | 含义 |
|------|------|
| `[M,CARn,vx,vy,vz]` | 实时速度控制（遥控，低延迟单发） |
| `[C,CARn,x,y,yaw]`  | 导航到目标位姿（可靠重发） |

### 4. 编队协议 `[F,...]`（服务器 -> 小车）

| 指令 | 含义 |
|------|------|
| `[F,S,leader,type]` | 启动编队，指定领航者与队形类型 |
| `[F,L,CARn]` | 指定该车为领航者 |
| `[F,F,leader,dx,dy,dyaw]` | 指定该车为跟随者及其相对领航者的偏移 |
| `[F,U,leader,dx,dy,dyaw]` | 运行中更新跟随偏移 |
| `[F,T]` | 停止编队 |

队形类型：`line`（一字）、`Diamond`（菱形）、`square`（方形）、自定义。
矩形轨迹实验中，领航者收到 `[T,leader,vx,vy,target_yaw]` 形式的巡航指令
（注意此 `[T,...]` 是矩形实验的领航速度指令，与下述拓扑协议前缀相同但语义不同，
由上下文区分：拓扑指令的第二字段是 `E`/`M`，矩形指令第二字段是车号）。

### 5. 通信拓扑协议 `[T,...]`（服务器 -> 小车广播）

用于控制“哪辆车能收到哪辆车的状态”，实现中心式/链式等通信结构。

| 指令 | 含义 |
|------|------|
| `[T,E,1]` / `[T,E,0]` | 启用 / 禁用拓扑过滤 |
| `[T,M,m00,m01,...,m33]` | 下发 4x4 拓扑矩阵（按行拉平，共 16 个 0/1） |

矩阵约定：`m[i][j]=1` 表示小车 j 能收到小车 i 的状态。下标映射
`CAR1->0, CAR2->1, CAR3->2, CAR4->3`。默认矩阵为中心式（仅 CAR1 向其余广播）。

### 6. 重构协议 `[R,...]`

重构分“准备 -> 拼接 -> 完成 ->（可选）分离”四个阶段，服务器为调度者。

服务器 -> 小车：

| 指令 | 含义 |
|------|------|
| `[R,ORDER,CAR1,CAR2,CAR3,CAR4]` | 下发完整拼接顺序 |
| `[R,PREP]` | 进入准备阶段（各车保持在前车后方固定间距） |
| `[R,ASM_START]` | 全员准备就绪，开始拼接 |
| `[R,STEP,CARn,POS=HEAD\|MID2\|MID3\|TAIL]` | 通知某车开始拼接及其目标位 |
| `[R,GUIDE,CARn,VX=..,VY=..,VZ=..,DONE=0\|1]` | 末端制导速度（约 20Hz） |
| `[R,SEP_STEP,CARn,POS=..]` | 分离阶段通知某车分离 |
| `[R,DONE]` | 全部拼接完成 |
| `[R,SEP_DONE]` | 全部分离完成 |
| `[R,ABORT]` | 中止重构 |

小车 -> 服务器：

| 报文 | 含义 |
|------|------|
| `[R,PREP_OK,CARn]` | 该车已就位（准备完成） |
| `[R,STEP_ACK,CARn]` | 该车已收到 STEP 并进入拼接 |
| `[R,STEP_OK,CARn]` | 该车拼接到位 |
| `[R,STEP_FAIL,CARn,REASON=..]` | 该车拼接失败 |
| `[R,SEP_OK,CARn]` | 该车分离完成 |
| `[R,SEP_FAIL,CARn]` | 该车分离失败 |
| `[R,DIAG,CARn,L=<级别>,C=<码>]` | 诊断信息（如 ASM_STAGGER_WAIT） |
| `[R,IMG_META,CARn,SEQ=..,W=..,H=..,LEN=..,TOT=..]` | 图像帧元信息（文本头） |

重要时序约定：

- 首车（order[0]）为锚点，不下发 STEP、不参与制导，等待被拼接。
- 拼接从第 2 辆开始，逐辆串行推进：STEP -> STEP_ACK -> 视觉制导 -> STEP_OK。
- STEP、GUIDE、ASM_START 等均以子网广播下发（小车按 `target==自身` 过滤），
  这是经现场验证可达的链路；单播仅用于遥控等低延迟场景。
- STEP 守护线程监控 STEP_ACK 与首帧图像超时，超时按上限重试，超限进入错误态。

### 7. 图像分片协议（小车 -> 服务器）

拼接阶段小车上报图传画面时采用“文本头 + 二进制分片”：

1. 先发文本头 `[R,IMG_META,CARn,SEQ,W,H,LEN,TOT,...]`。
2. 再发若干二进制分片，每片格式为：

```
[R,IMG_CHUNK,CARn,SEQ=..,IDX=..,TOT=..,SZ=..]\n<原始JPEG字节>
```

服务器以 `\n` 作为头与二进制负载的边界，先对头做 UTF-8 解码，`\n` 之后为
原始 JPEG 字节，不做二次编码。以 `(car_id, SEQ)` 作为帧命名空间，按
`IDX/TOT/LEN` 重组，避免跨车混拼。

注意：当前部署以“采集卡本地图传 + 服务器端检测”为主视觉链路，图像分片协议
作为契约保留（详见 `docs/reconstruct_contract.md`）。

---

## 六、HTTP 接口清单

服务器 Web 端口默认 5000。前端页面 `templates/index.html` 通过以下接口交互。

### 页面与小车

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | Web 控制台首页 |
| GET | `/api/cars` | 所有小车状态快照 |

### 广播

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/broadcast` | 开关周期广播 |
| POST | `/api/broadcast/interval` | 设置广播间隔 |
| POST | `/api/broadcast/group_size` | 设置广播分组大小 |

### 单车控制

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/control_position` | 导航到目标位姿（编队/矩形实验时拦截） |
| POST | `/api/control_velocity` | 实时速度遥控（编队时仅允许领航者） |

### 拓扑

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/topology` | 设置拓扑矩阵并启用/禁用 |
| GET | `/api/topology/status` | 查询当前拓扑 |
| POST | `/api/topology/toggle` | 仅启用/禁用拓扑过滤 |
| GET | `/api/topology/visible/<car_id>` | 查询某车可见的僚车 |

### 重构

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/reconstruct/start` | 启动重构（下发 ORDER + PREP） |
| POST | `/api/reconstruct/separate` | 拼接完成后启动分离 |
| POST | `/api/reconstruct/abort` | 中止重构并恢复拓扑 |
| GET | `/api/reconstruct/status` | 状态机快照 |
| GET | `/api/reconstruct/events` | 每车事件时间戳 |
| GET/POST | `/api/reconstruct/params` | 查询/设置可调参数 |
| GET | `/api/reconstruct/image` | 当前监控图像（base64） |
| GET | `/api/reconstruct/logs` | 内存中的调试日志 |
| GET | `/api/reconstruct/logfile` | 按日持久化的 jsonl 日志 |

### 视觉

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/vision/image` | 监控图像（base64，供弹窗） |
| POST | `/api/vision/select` | 选择监控目标车 |
| GET | `/api/vision/status` | 视觉状态（监控目标/绑定关系/告警） |
| POST | `/api/vision/bind/scan` | 重新扫描并绑定摄像头 |
| GET | `/stream/mjpeg/<car_id>` | MJPEG 实时视频流 |

### 位姿与编队

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/save_pose` | 定时保存位姿数据（10/20/30 秒） |
| POST | `/api/formation/start` | 启动编队 |
| POST | `/api/formation/stop` | 停止编队 |
| GET | `/api/formation/status` | 编队状态 |
| POST | `/api/formation/custom` | 自定义编队偏移 |
| GET | `/api/formation/configs` | 预设队形配置 |
| POST | `/api/formation/update_offsets` | 运行中更新偏移 |
| POST | `/api/formation/rectangle/start` | 启动矩形轨迹实验 |
| POST | `/api/formation/rectangle/stop` | 停止矩形轨迹实验 |
| GET | `/api/formation/rectangle/status` | 矩形实验状态 |
| GET | `/api/formation/rectangle/download/<filename>` | 下载轨迹 CSV |

---

## 七、运行与部署

### 环境依赖

```
pip install -r requirements.txt
```

另需 OpenCV（含 aruco 模块）、numpy、flask、flask-cors；建议安装 netifaces
以便自动探测子网广播地址。视觉功能需要接入采集卡/摄像头。

### 端口

| 端口 | 用途 |
|------|------|
| 5000 | Flask Web 控制台 |
| 8080 | UDP 遥测/指令接收 |
| 8081 | UDP 子网广播 / 小车指令监听 |

### 启动

```
python web_car_server.py
```

启动流程：统一控制台编码 -> 打印网络信息、刷新拓扑缓存 -> 创建 UDP 服务器
单例 -> 初始化本地视觉（交互选择部署车辆并绑定摄像头，可跳过）->
启动 UDP 服务器与各守护线程 -> 装配并注册 Flask 蓝图 -> 运行 Web 服务。

启动后浏览器访问 `http://<服务器IP>:5000`。

### 安全说明

Web 控制台与 UDP 接口默认无鉴权，且监听 `0.0.0.0`（对局域网开放）。
仅应在可信实验网络中使用；若需暴露到更大范围，请自行增加访问控制。

---

## 八、协同开发分工建议

模块已按功能域解耦，建议按模块边界分工，减少合并冲突：

- 网络与遥测：`net.py`、`state.py`、`routes/cars.py`、`routes/broadcast.py`、
  `routes/control.py`、`routes/topology.py`、`topology.py`。
- 重构与制导：`reconstruct.py`、`guide.py`、`routes/reconstruct.py`。
- 视觉与前端：`vision.py`、`Car_vision_system/`、`routes/vision.py`、
  `templates/index.html`。
- 编队独立成模块：`formation_controller.py`。

跨模块协作规则：

1. 修改共享状态字段前，先在 `state.py` / `config.py` 的定义处沟通并加注释。
2. 新增小车指令协议时，同步更新本文档第五节，保持服务器与小车端一致。
3. 遵循“共享可变量必须经 `模块名.变量` 访问”的约定（见第三节）。
4. 新增 HTTP 接口时，放入对应业务域的蓝图文件，并在本文档第六节登记。
5. 涉及锁的改动务必留意第三节的线程表，避免死锁或竞态。

历史单体实现保留在 `web_car_server_legacy_backup.py`，如需核对重构前的行为
可作对照，但它不参与运行。
