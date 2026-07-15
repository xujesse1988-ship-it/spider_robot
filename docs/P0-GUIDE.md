# P0 准备阶段 · 详细操作指南

目标（1~2 周）：**第一批器件到货，Pi 5 软件环境全绿，一条腿装配完成并能由 Pi 控制摆动。**
本阶段结束时你应该能拍出一段视频：SSH 进 Pi 敲命令，桌上的一条蜘蛛腿抬起又放下。

各步骤可并行：第 1 步（下单）当天做完，等快递期间做第 2 步（Pi 环境）和第 3 步（打印）。

---

## 第 1 步 · 下单第一批器件（第 1 天）

按 `BOM.md` 表 1/表 2 的规格执行，本批清单与下单前必须和卖家确认的点：

| # | 器件 | 数量 | 下单前确认 |
|---|---|---|---|
| 1 | 35kg·cm 数字舵机（DS3235/ZOSKAY 级） | 4 | ①**180° 控制角**版本（不是 270°）②工作电压含 7.4V ③带舵盘和螺丝 |
| 2 | Pimoroni Servo2040 | 1 | 正品（代购/海淘，周期最长，最先下单） |
| 3 | 5V/5A DC-DC 降压模块 | 1 | 输入范围含 6~8.4V；带 USB-C 口的更方便直连 Pi |
| 4 | 2S 锂电 5000mAh+ XT60 + B3 充电器 | 各1 | 电池带保护板或自备低压报警器 |
| 5 | 555 双头真空泵 12V | 1 | 极限真空 ≤-75kPa |
| 6 | 二位三通电磁阀 12V | 2 | **常闭型**（断电=气路断开保持真空）——必须文字确认 |
| 7 | 30mm 2.5折波纹真空吸盘 | 3 | 配 **M5 通孔金具 + 90° 弯头宝塔**（气从螺柱中心走） |
| 8 | XGZP6847A 压力传感器 0~-100kPa | 1 | 模拟输出版（不是 I2C 版） |
| 9 | ADS1115 模块 | 1 | — |
| 10 | 8 路 MOSFET 驱动板 | 1 | 光耦隔离版更好 |
| 11 | XL6009 升压模块 | 1 | — |
| 12 | 4×6mm 硅胶管 5m + 4mm 三通/直通/单向阀 | 1套 | — |
| 13 | M1.6×6 内六角螺丝 100 装、M2.5×6、M3 | 各1包 | M1.6 是舵盘-打印件连接的主力 |
| 14 | 杜邦线、XT60 对插、16AWG 硅胶线、热缩管 | 若干 | — |

预算合计约 ¥600~950。第二批（15 个舵机等）**此时不买**。

## 第 2 步 · Pi 5 软件环境（第 1~3 天，无需任何硬件）

1. **烧录系统**：官网 Raspberry Pi Imager → 选 **Raspberry Pi OS Lite (64-bit)** →
   点齿轮预配置：主机名 `spider`、开 SSH、填 WiFi、用户名密码。
2. **首次登录与基础配置**：
   ```bash
   ssh <用户名>@spider.local
   sudo apt update && sudo apt install -y git python3-venv i2c-tools
   sudo raspi-config nonint do_i2c 0        # 开 I2C（ADS1115 用）
   sudo usermod -aG dialout $USER            # 串口权限（Servo2040 用）
   # 重新登录使组权限生效
   ```
3. **部署本仓库**：把开发机上的仓库推到你的 git 远端后在 Pi 上 clone，
   或直接 `rsync -a --exclude .venv --exclude .git ~/spider/ <用户名>@spider.local:~/spider/`。
4. **安装并验证软件包**：
   ```bash
   cd ~/spider/software
   python3 -m venv .venv && source .venv/bin/activate
   pip install -e ".[sim,dev]"
   pytest tests/                             # 期望：18 passed
   python scripts/sim_walk.py --gif /tmp/walk.gif --seconds 2
   python scripts/stand_up.py --mock         # 干跑，Ctrl-C 退出
   ```
   把 `/tmp/walk.gif` scp 回来看一眼：六足三角步态动画即为通过。
5. **供电检查习惯**（以后每次都做）：`vcgencmd get_throttled` 输出必须是 `0x0`，
   非零说明欠压/降频，先解决供电再继续。

## 第 3 步 · 3D 打印（第 1~4 天，与快递并行）

打印一条**左前腿 L1**（选左前是因为软件默认配置的测试通道就是它：15/16/17）：

| 文件（`hardware/makeyourpet-hexapod/STL/`） | 数量 | 说明 |
|---|---|---|
| `left-coxa2.stl` | 1 | 同名多版本取编号最大（coxa2 > coxa） |
| `left-femur.stl` | 1 | — |
| `hardware/climbing-parts/left-tibia-suction.stl` | 1 | 吸盘足一体化小腿（替代 left-tibia.stl，末端腔体直接卡吸盘） |
| `tip.stl` | 1 | 普通足尖（P0 摆动测试用，可套在旧 left-tibia 上） |
| `calibration-arm.stl` + `calibration-ruler.stl` | 各1 | 官方舵机标定工具，P1 标定 ±45° 脉宽要用 |
| `hardware/climbing-parts/suction-foot-door.stl` | 1 | 吸盘腔门盖（吸盘放入后 502 粘死） |

参数：coxa/femur 用 PLA、4 壁、40% 填充、0.2mm 层高；`left-tibia-suction.stl`
建议 PETG、5~6 壁、45% 填充，**立打**（吸盘腔朝下）+ brim ≥8mm，tibia 细节可开支撑；
`suction-foot-door.stl` 外平面朝下、无支撑。腿部件的摆放方向和是否加支撑，跟着官方
视频走（YouTube 频道 **MakeYourPet**，装腿是第 1 集）；装配小件（limiter、
servo-back-hole2 等）按视频里单腿所需补打。
打完先干配一次：吸盘连金具从正面推入腔体，凸筋应"咔"进 Ø15 凹槽、两颗螺母落进
六角袋不能转动，盖上门无明显旷量；过紧/过松改 `tools/generate_climbing_parts.py`
的 `gap_d`/`gap_hex`（±0.2mm）重打。确认无误后再上 502 粘门。

## 第 4 步 · Servo2040 刷固件（到货当天，5 分钟）

1. 下载社区固件：
   https://github.com/EddieCarrera/chica-servo2040-simpleDriver/releases/download/v0.0.1/chica-servo2040_release.uf2
2. **按住板上 BOOT 键**插 USB → 电脑出现 `RPI-RP2` U 盘 → 把 .uf2 拖进去 → 自动重启完成。
3. 验证：插到 Pi 上，`ls /dev/ttyACM*` 应出现设备；然后：
   ```bash
   cd ~/spider/software && source .venv/bin/activate
   python -c "from hexapod.driver import Servo2040Driver; d=Servo2040Driver(); print('电压', d.read_voltage_v()); d.close()"
   ```
   此时未接舵机电源，读数取决于板子背面 "Separate USB & Ext. Power" 跳线：
   未割断（出厂默认）读数约 5V（读到的是 USB 的 5V 供电轨）；已割断则接近 0。
   两种情况下能打印出数字，就说明 USB 通信链路通了。
   ⚠️ 第 5 步接 2S 电池**之前必须先割断该跳线**（接线图上标注 "CUT THIS!"），
   否则电池的 7.4~8.4V 会经 USB 线倒灌进 Pi，可能烧毁 Pi。

## 第 5 步 · 单腿装配（舵机到货后，半天）

⚠️ 顺序很重要：**先让舵机回中，再装舵盘，最后装结构件**，装反了标定全错。

1. **接电池前的强制检查**（三项全部通过才允许接电池，任何电源都没接时先做第一项）：
   - [ ] **割断**板背面 "Separate USB & Ext. Power" 跳线（接线图上标注 "CUT THIS!"）。
         不割就接 2S 电池，电池电压会经 USB 线倒灌烧毁 Servo2040 和 Pi 的 USB 口。
   - [ ] **验证割断**：只插 USB、不接电池，跑第 4 步的读电压命令——
         读数**接近 0** 才算割断成功；仍读到约 5V 说明没割透，回去补割再测。
   - [ ] **万用表实测**电池引出线/XT60 的正负极后再接 EXT 端子，**不要只看线色**
         （自购引出线偶有红黑接反，反接瞬间烧板）。

   养成习惯：接/拔电池前先拔 USB，两路电源永远不同时插拔。
2. **接线**（对照 `hardware/makeyourpet-hexapod/wiring-diagram-servo2040.png`）：
   3 个舵机信号线接 Servo2040 通道 **15(coxa) / 16(femur) / 17(tibia)**；
   舵机电源按官方接线图接 2S 电池（P0 可暂不装继电器，直连，但要装 XT60 快断）。
3. **回中**：
   ```bash
   python scripts/servo_center.py            # 全部通道回中 1500µs 并使能
   ```
   听到舵机锁定即成功。此时**不要**用手掰舵机。
4. **装舵盘**：按官方视频的中位角度把舵盘压上花键、上锁紧螺丝——
   coxa/femur/tibia 的中位姿态分别对应官方安装偏角 -8°/35°/68°（视频里有对齐示意）。
5. **装结构件**：coxa → femur → tibia 依次装配（视频第 1 集），M1.6 螺丝连接舵盘与打印件。

## 第 6 步 · 摆动测试与验收（1 小时）

```bash
python scripts/servo_center.py                       # 回中基准
python - <<'EOF'                                     # 三关节各扫一遍 ±20°
import time
from hexapod.driver import Servo2040Driver
d = Servo2040Driver()
d.set_all_pulses_us([1500]*18); d.enable(True); time.sleep(1)
for ch in (15, 16, 17):
    for us in (1280, 1720, 1500):
        d.set_pulses_us(ch, [us]); time.sleep(0.8)
d.close()
EOF
```

**P0 验收核对表**：

- [ ] 第一批器件全部到货，常闭阀/180°舵机/M5通孔金具经实物核对
- [ ] Pi：`pytest` 18 项全过、`sim_walk.py` 出 GIF、`get_throttled` = 0x0
- [ ] Servo2040 刷好固件，Pi 能读电压
- [ ] 电源跳线已割断（只插 USB 读电压接近 0），电池极性经万用表确认
- [ ] 单腿三关节按指令平滑摆动，无抖动、无异响、结构无松动
- [ ] 吸盘+金具推入 `left-tibia-suction.stl` 腔体到位，螺母入六角袋不转动，门盖干配无旷量
- [ ] 整腿称重记录（写进 `docs/weight-log.md`，预算参考：单腿含舵机 ~230g）

全部勾选 → 进入 P1（吸附台架验证）。

## 常见问题

| 症状 | 处理 |
|---|---|
| `/dev/ttyACM0` 不存在 | 换数据线（很多 USB-C 线是纯充电线）；确认固件刷成功（板上 LED 彩虹流动=等待连接） |
| 打开串口 Permission denied | `dialout` 组没生效，重新登录；或临时 `sudo chmod 666 /dev/ttyACM0` |
| 接电池后板子/Pi 冒烟、无响应 | 大概率跳线没割断致电池电压倒灌，或电池极性接反。立即断电池拔 USB；换一台电脑按 BOOT 测板子是否还能出 `RPI-RP2`；Pi 逐口插已知好设备验证 USB 口 |
| 舵机不动但通信正常 | 忘了 `enable(True)`（RELAY 使能）；或舵机电源没接/电池没电 |
| 舵机抖动/嗡嗡响 | 2S 电压低于 6.8V 先充电；信号线与电源线分开走；单条腿负载下抖动属异常，换备用舵机排查 |
| 回中后舵盘角度装不准 | 舵盘花键有齿距，装最接近的一格，残差以后用 `config.py` 的 `attach_deg` 修 |
| Pi 频繁重启/降频 | `get_throttled` 非 0：换 5V/5A 供电，别用电脑 USB 口带 Pi |
| 吸盘推不进腔体/螺母卡不进六角袋 | 打印件 XY 公差偏紧：`gap_d`/`gap_hex` 加 0.2 重打，或用锉刀修；过松晃动则减 0.2 |

## 安全

- 锂电池充电不离人，用 B3/B6 平衡充；测试台上装 XT60 快断，异响立即断电。
- 35kg 舵机堵转扭矩很大：通电状态手指不要放进关节转动范围。
