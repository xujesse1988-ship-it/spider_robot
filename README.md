# Spider —— DIY 爬墙多足机器人

手工 DIY 一台类蜘蛛的六足爬行机器人，目标是光滑墙面（玻璃/瓷砖）爬行 + 负重。开发顺序按风险前置：**先用一条腿验证"上墙"可行（P0~P2 决策门，只花小几百块），再装整机走地面（P3），最后整机上墙（P4）**。大脑为树莓派 Pi 5（USB 驱动 Servo2040 舵机板，控制软件自研，见 `software/`）。

## 文档

| 文件 | 内容 |
|---|---|
| [docs/ROADMAP.md](docs/ROADMAP.md) | 分阶段实现路线图 P0~P5（含每阶段验收标准与风险清单） |
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
| `suction_foot.stl` | 吸盘足模块：套接 MakeYourPet 腿末端 9×9mm 方轴（顶丝锁紧），底部装 30mm 波纹真空吸盘（M5 通孔金具+弯头宝塔），侧窗出气管。打印 6 件 |
| `component_plate.stl` | M3 网格安装板，固定真空泵/电磁阀/传感器 |

打印建议：PETG 或 PLA，4 壁、40% 填充；`suction_foot` 吸盘面朝下打印无需支撑。

尺寸与实购件不符时（如吸盘螺柱直径），改 `tools/generate_climbing_parts.py` 顶部 `PARAMS` 后重新生成：

```bash
python3 -m venv .venv && .venv/bin/pip install trimesh manifold3d numpy scipy shapely
.venv/bin/python tools/generate_climbing_parts.py
```

## 快速开始

1. 读 `docs/ROADMAP.md` 的 P0~P2 节（单腿上墙验证是第一目标，也是决策门）。
2. 按 `docs/BOM.md` 开头的**分批采购**下单第一批（单腿验证套件，约 ¥600~950）。
3. 等快递期间打印一条腿（`hardware/makeyourpet-hexapod/STL/` 的 coxa/femur/tibia，同名多版本取编号最大的）+ 1 件 `climbing-parts/suction_foot.stl`，看上游 YouTube 装配视频（频道：MakeYourPet）。
4. 给 Pi 5 烧 Raspberry Pi OS Lite 64-bit，装 `software/` 包，跑 `scripts/sim_walk.py --gif walk.gif` 看步态仿真。
5. P2 决策门（单腿 50 次吸放循环 >95%、撑住 1.5kg 配重）通过后，再买第二批装整机。

## 安全

- 爬墙测试全程系安全绳、地面铺垫；锂电池充电不离人。
