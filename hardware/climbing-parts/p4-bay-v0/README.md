# P4 电气+气动舱 V0

本目录由 `tools/generate_p4_bay_v0.py` 生成。V0 是**实物试装版**,不是未经
验证即可爬墙的终版。全部 13 个 STL 已通过水密、有限顶点和正体积检查；
孔位、尺寸与打印数量见 `layout-report.json`。

`p4-bay-v0-lower-layout.png` 和 `p4-bay-v0-assembly.png` 是包络检查预览；
其中彩色方块/圆柱只是按实测外形或保守高度建立的器件代理,不是打印件。

按下表数量、不含 1mm 模板时,几何实体体积约 237.2cm³；全实心 PETG/TPU
质量上限约 301g。实际切片会因填充率降低,但必须以打印后称重为准。V0 用于
试装；通过后还要做 V1 镂空/薄壁减重,目标打印固定件不超过 180g。

## 必须先打印的件

先打 `p4-bay-fit-template-v0.stl`。它是 190×130×1mm 外廓模板,中央开口和
frame 接口与正式底板一致。只用原有 `(±44,±40)` 四孔装到 frame,装好六个
coxa 舵机和线夹后做全行程检查。任何一处擦线或碰舵机,都应先改底板外廓,
不要继续打印正式底板。

`(±44,0)` 两孔只是可选加强位。确认其下方为实心且确有必要后,才给 frame
钻 Ø2.8mm M3 自攻底孔。

## STL 与数量

| 文件 | 数量 | 材料 |
|---|---:|---|
| `p4-bay-fit-template-v0.stl` | 1 | PLA/PETG |
| `p4-bay-baseplate-v0.stl` | 1 | PETG |
| `p4-bay-electrical-deck-v0.stl` | 1 | PETG |
| `p4-bay-deck-post-90mm-v0.stl` | 6 | PETG,仅试装 |
| `p4-bay-spacer-m25-6mm-v0.stl` | 4 | PLA/PETG |
| `p4-bay-spacer-m3-6mm-v0.stl` | 按需约 20 | PLA/PETG |
| `p4-bay-pump-adapter-v0.stl` | 1 | PETG |
| `p4-bay-pump-pad-tpu-v0.stl` | 1 | TPU 95A |
| `p4-bay-valve-rail-v0.stl` | 1 | PETG |
| `p4-bay-sensor-block-v0.stl` | 1 | PLA/PETG,4+3 双排 |
| `p4-bay-manifold-clip-v0.stl` | 2 | PETG |
| `p4-bay-switch-bracket-v0.stl` | 1 | PETG |
| `p4-bay-cable-comb-v0.stl` | 4 | PLA/PETG |

## 建议打印与紧固

- 0.20mm 层高、4 道墙、5 层顶/底、30%~40% gyroid；底板和甲板平躺。
- 泵用 4×M4、大垫圈和 3mm TPU 垫；螺丝只压到不窜动,不要压死 TPU。
- Pi 用 M2.5；其他 PCB 用 M3；所有 PCB 与甲板之间装 6mm 隔柱。
- 阀轨每只阀用一条 2.5mm 扎带；传感器试插合适后用少量中性硅胶点固。
- 90mm 打印柱用于确认高度和孔位。正式爬墙优先改用 M3×90 金属/尼龙
  六角柱或两段组合柱,并使用防松螺母。
- 储气罐尺寸尚未实测,本目录没有罐抱箍；到货后再生成 V1。

重新生成:

```bash
venv/bin/python tools/generate_p4_bay_v0.py
```
