# Spider —— DIY 爬墙多足机器人

手工 DIY 一台类蜘蛛的六足爬行机器人，目标是光滑墙面（玻璃/瓷砖）爬行 + 负重。开发顺序按风险前置：**先用一条腿验证"上墙"可行（P0~P2 决策门，只花小几百块），再装整机走地面（P3），最后整机上墙（P4）**。大脑为树莓派 Pi 5（USB 驱动 Servo2040 舵机板，控制软件自研，见 `software/`）。

## 文档

| 文件 | 内容 |
|---|---|
| [docs/ROADMAP.md](docs/ROADMAP.md) | 分阶段实现路线图 P0~P5（含每阶段验收标准与风险清单） |
| [docs/P0-GUIDE.md](docs/P0-GUIDE.md) | P0 准备阶段逐步操作指南（下单核对、Pi 环境、打印、单腿装配与验收） |
| [docs/P1-GUIDE.md](docs/P1-GUIDE.md) | P1 吸附系统台架验证指南（真空回路、三条曲线、挂重/斜角测试） |
| [docs/P2-GUIDE.md](docs/P2-GUIDE.md) | P2 单腿爬墙决策门指南（**2026-07-22 已通过**，含验收数据） |
| [docs/P3-GUIDE.md](docs/P3-GUIDE.md) | P3 整机装配与地面行走指南（第二批下单、打印盘点、全舵机标定） |
| [docs/BOM.md](docs/BOM.md) | 分阶段采购清单（含淘宝搜索关键词与价格区间） |
| [docs/CLIMBING-DESIGN.md](docs/CLIMBING-DESIGN.md) | 吸附方式选型论证、力学预算、气路图、电气架构 |

## 3D 打印文件

| 目录 | 说明 |
|---|---|
| `hardware/makeyourpet-hexapod/STL/` | 步行平台：MakeYourPet 六足全套 STL（MIT 协议，含 STEP 源文件、接线图；上游 https://github.com/MakeYourPet/hexapod ） |
| `hardware/climbing-parts/` | **本项目自制爬墙部件**（由 `tools/generate_climbing_parts.py` 参数化生成，见下） |

## 控制软件（software/）

Pi 5 上运行的自研 Python 包：单腿 IK/FK、三角/波浪/爬墙步态引擎、Servo2040 chica 串口协议驱动（已按固件源码逐字节核实）、吸附状态机 + 真空回路仿真。无硬件即可跑仿真动画和 18 项测试，详见 [software/README.md](software/README.md)。

### 自制爬墙部件（hardware/climbing-parts/）

| 文件 | 用途 |
|---|---|
| `left-tibia-suction.stl` | 吸盘足 v3：与 MakeYourPet left-tibia **一体化**的小腿（替代 left-tibia.stl + 旧吸盘足，腿短约 59mm）。末端腔体按实购"2.5 折吸盘+双六角螺母+直角宝塔"一体件（`images/xipan_marked.jpeg`）做全形状负模：Ø27 折痕座圈+5°肩面 → Ø17 主孔 → Ø15 凹槽凸筋 → 六角袋锁转动 → 弯头腔。打印 3 件（右腿版待生成） |
| `suction-foot-door.stl` | 上述腔体的门盖：吸盘放入后盖上、502 粘接（含前半负模 + 4 定位销），宝塔嘴从门顶上方开口伸出、可转动 |
| `component_plate.stl` | M3 网格安装板，固定真空泵/电磁阀/传感器 |

打印建议：PETG 优先；`left-tibia-suction` **立打**（吸盘腔朝下、tibia 朝上，5~6 壁、45% 填充、开 brim ≥8mm，tibia 细节可开支撑保险）——方轴轴线距 tibia 平背面仅 4.5mm 而腔体半径 17mm，平躺必穿打印床，几何上只能立打；`suction-foot-door` 外平面朝下平躺、无支撑。

尺寸与实购件不符时（如吸盘颈部直径），改 `tools/generate_climbing_parts.py` 顶部 `PARAMS` 后重新生成：

```bash
python3 -m venv .venv && .venv/bin/pip install trimesh manifold3d numpy scipy shapely
.venv/bin/python tools/generate_climbing_parts.py
```

## 快速开始

1. 读 `docs/ROADMAP.md` 的 P0~P2 节（单腿上墙验证是第一目标，也是决策门）。
2. 按 `docs/BOM.md` 开头的**分批采购**下单第一批（单腿验证套件，约 ¥600~950）。
3. 等快递期间打印一条腿：coxa/femur 用 `hardware/makeyourpet-hexapod/STL/`（同名多版本取编号最大的），tibia 用 `climbing-parts/left-tibia-suction.stl`（已集成吸盘足）+ 1 件 `suction-foot-door.stl`，看上游 YouTube 装配视频（频道：MakeYourPet）。
4. 给 Pi 5 烧 Raspberry Pi OS Lite 64-bit，装 `software/` 包，跑 `scripts/sim_walk.py --gif walk.gif` 看步态仿真。
5. P2 决策门（单腿 50 次吸放循环 >95%、撑住 1.5kg 配重）通过后，再买第二批装整机。

## 安全

- 爬墙测试全程系安全绳、地面铺垫；锂电池充电不离人。
