# 自制爬墙部件（hardware/climbing-parts/）

全部由 [`tools/generate_climbing_parts.py`](../../tools/generate_climbing_parts.py) 参数化生成。协议与第三方派生说明见 [NOTICE.md](NOTICE.md)。

| 文件 | 用途 |
|---|---|
| `left-tibia-suction.stl` | **定版吸盘小腿**：与 MakeYourPet left-tibia 一体化（替代 left-tibia.stl + 旧吸盘足，腿短约 59mm）。末端腔体按实购"2.5 折吸盘+双六角螺母+直角宝塔"一体件（`images/xipan_marked.jpeg`）做全形状负模：Ø27 折痕座圈 + 45° 肩面 → Ø17 主孔 → Ø15 凹槽凸筋 → 六角袋锁转动 → 弯头腔。打印 3 件 |
| `right-tibia-suction.stl` | 定版右腿吸盘小腿：由左腿沿 XZ 平面镜像生成，与官方左右 tibia 的变换一致；打印 3 件，配同款 `suction-foot-door.stl` |
| `suction-foot-door.stl` | 定版吸盘腔门盖：含前半负模、4 个定位销和 2 个 M5 沉头过孔，左右腿共用；宝塔嘴从门顶开口伸出、可转动 |
| `component_plate.stl` | M3 网格安装板，固定真空泵/电磁阀/传感器 |
| `coxa-pedestal.stl`、`carriage-pedestal.stl` | P2 单腿台架用的支座与滑车一体件（`html/coxa-pedestal-mount.html`） |
| `p4-bay-v0/`、`p4-bay-v1/` | P4 电气+气动舱固定件（设计见 `docs/P4-BAY-DESIGN.md`，v0 目录内有装配图与 README） |
| `*-exp.stl` | 带触地环的试验版，未采用 |

## 打印建议

- PETG 优先。
- 左右 `tibia-suction` 均**立打**（吸盘腔朝下、tibia 朝上，5~6 壁、45% 填充、开 brim ≥8mm，tibia 细节可开支撑保险）。原因：方轴轴线距 tibia 平背面仅 4.5mm 而腔体半径 17mm，平躺必穿打印床，几何上只能立打。
- `suction-foot-door` 外平面朝下平躺、无支撑。

## 尺寸不符时重新生成

吸盘颈部直径之类与实购件不符时，改脚本顶部 `PARAMS` 后重新运行：

```bash
python3 -m venv .venv && .venv/bin/pip install trimesh manifold3d numpy scipy shapely
.venv/bin/python tools/generate_climbing_parts.py
```
