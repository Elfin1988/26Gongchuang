# WS 上位机控制台

这是一个放在 `ws/` 目录下的独立上位机工程，采用：

- `FastAPI` 作为本地后端
- `pyserial` 负责串口访问
- 原生 `Web UI` 负责控制面板和日志展示

## Python 版本

建议使用 `Python 3.8+`。

当前依赖已经按 `Python 3.8` 做了兼容选择。

## 当前能力

- 枚举本机串口
- 打开 / 关闭串口
- 按竞赛最小版协议发送命令包
- 实时接收串口字节流
- 实时解析协议帧并展示
- 同时保留原始串口日志与手动 HEX / 文本发送能力

## 安装依赖

```bash
cd ws
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Linux/macOS:

```bash
cd ws
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

如果页面能打开，但日志里出现下面这类提示：

```text
No supported WebSocket library detected
GET /ws HTTP/1.1 404 Not Found
```

说明当前虚拟环境缺少 WebSocket 运行依赖。重新执行：

```bash
pip install -r requirements.txt
```

或单独安装：

```bash
pip install websockets==12.0
```

## 启动

```bash
cd ws
uvicorn app.main:app --reload
```

打开浏览器访问：

```text
http://127.0.0.1:8000
```

## Linux 说明

- 串口设备通常会显示为 `/dev/ttyUSB0`、`/dev/ttyACM0` 或 `/dev/serial/by-id/...`
- 推荐优先选择 `/dev/serial/by-id/...` 这种稳定路径
- 如果发现串口但无法打开，通常是权限问题，可将当前用户加入 `dialout` 组

示例：

```bash
sudo usermod -aG dialout $USER
```

然后重新登录。

## 目录结构

```text
ws/
  README.md
  requirements.txt
  app/
    main.py
    protocol.py
    serial_manager.py
    static/
      app.js
      styles.css
    templates/
      index.html
```

## 注意

- 当前仓库里的 STM32 固件还存在旧版“单字节指令 + 文本上报”逻辑。
- 本控制台已经支持原始串口模式，便于在下位机完全切换到新协议之前先联调。
