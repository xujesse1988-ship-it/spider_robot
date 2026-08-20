# vent 前零力交接（zero-force handover）· 实现设计

状态：**设计定稿，待实现**（2026-08-20；实现排在下一个会话）。
实测依据：`html/vent-snap-20260820.html`（08-20 原地踏步实验量化报告）、
`software/logs_analysis/lean_20260820_102117.log`、`images/lean_20260820.mp4`。

---

## 1. 要解决的问题（一段话）

零指令原地逐腿抬落，机器人每轮沿墙下滑 57→74mm；逐帧量化显示 **83% 的下滑
发生在放气密封破裂的一瞬间**（阀开后 0.4s 泄压期身体位移 <0.15mm，破裂后
<100ms 内一步坠到位，带过冲振铃）——被抬腿自吸附以来储存的弹性势能
（身体每下沉 1mm，该腿被绕紧 1mm）在脱吸瞬间释放，身体坠到其余五腿的新增
变形接住为止，随后被下一次吸附锁死成棘轮。press 落地只占 4%（vent-first
时序下 press-lurch 已退居次要）。整机电流 0.73→~1.9A 逐周期爬升不回落，
是腿间内应力累积的电学证据。**慢放气已被数据否决**：力的释放不随泄压渐进，
全部压缩在唇口脱开一瞬——治法只能是让脱开时刻这条腿本来就不带力。

单次弹跳按腿（第二轮，mm）：L1 20.9 / R1 19.2 / L3 13.5 / R3 11.8 /
R2 5.9 / L2 5.8。前腿（上方腿，剥离力矩集中）最大。单腿弹跳封顶 ~21mm
（舵机扭矩饱和限制单腿储能），这解释了爬行仍能净前进 26% 而非倒退。

## 2. 原理

**吸住的脚不能滑 ⇒ 改指令 = 改力，不改位。** 这是整个方案的物理基础，与
现有 VENT 段"贴面随支撑场平移"完全同一性质（引擎已有先例，update() 第 4 步）。

一维弹簧账（每腿一根弹簧 k，y_i = 锚点 − 指令 = 该腿"暗示"的身体位置，
X = mean(y_attached) − mg/(n·k)）：抬腿 j 无跳变的充要条件是 **f_j = 0**，
即 y_j = mean(其余) − mg/5k。做法：**把被抬腿的足端指令沿"上坡"方向还 δ
（卸载自己），同时其余五条支撑腿的指令各沿"下坡"方向多走 δ/5（接住载荷）**
——六腿 y 的均值不变 ⇒ 身体纹丝不动；y_j 单独降 δ ⇒ f_j 清零。随后放气，
密封破裂时无能量可释放。1-D 模型验证（附录 A）：基线净前进 25.8%（与实测
26% 咬合）→ 零力交接后 100%；原地踏步 −35.6mm/周期 → 0。

三个工程性质：

- **其余腿承担的力不超过抬腿后本来就要承担的量**——交接只是把载荷移交提前、
  并做成无损，没有新的最坏工况；
- 指令几何自洽有界：某腿在一个周期里作为支撑腿吃 5 次 δ/5（共 −δ 下坡），
  轮到自己抬时先 +δ 卸载正好抵回站位附近，落点又是站位——指令不发散，
  单腿最大额外外摆 ≈ δ（工作空间账见 §4.7）；
- 方向沿 `_down`（下坡方向，随积分航向旋转，与 sag_comp 同一套）：本实验
  横向漂移仅 7.6mm/131mm，纵向单轴假设成立。

## 3. 机构设计

### 3.1 配置

- `LegConfig.handover_mm: float = 0.0`——逐腿交接量 δ（0=关，默认关；与
  `press_delta_mm` 同为逐腿字段）。放 LegConfig 而不是 RobotConfig：实测
  逐腿差 4 倍（L1 17 vs R2 5），全局一个值没法用。
- `climb.py` 模块常量 `HANDOVER_SPEED_MMS = 10.0`——交接铺设速率，与
  LEAN_SPEED_MMS 同量级（吸住的脚改指令要慢，给载荷重分配留准静态时间）。
  δ=17 时交接段 ~1.7s。

### 3.2 新相位

`LegPhase.HANDOVER = "handover"`，插在窗头决策与 VENT 之间：

```
STANCE →(窗头决策放行)→ HANDOVER →(δ 铺完, request_release)→ VENT → LIFT → …
```

⚠ `runlog.PHASE_CH` 必须加 `"handover": "Z"`（零力）——status_line 和
ClimbWatch 遥测都查这张表，漏加 = KeyError 炸主环/黑匣子。ClimbWatch 的
相位跳变日志（`相位 X stance→handover`）走 `.value` 通用路径，自动生效。

### 3.3 窗头决策改动（update() 第 2 步 else 分支）

现有该分支做的事**全部保留原位**：`landing` 计算、轮次推进（step_leg）、
`_swung_since_go` 记账、sag_comp 装填、`step_pending→step_active`、
`_slot_active = True`。唯一改动是收尾动作分叉：

```python
if self.cfg.leg(cur).handover_mm > 0.0:
    self._ho_left = self.cfg.leg(cur).handover_mm
    self.phase_of[cur] = LegPhase.HANDOVER      # 先交接，不放气
else:
    self.ctl.request_release(LEG_NAMES.index(cur))
    self.phase_of[cur] = LegPhase.VENT          # 现行为，一字不动
```

`request_release` **推迟到交接完成**才调用——交接期间吸盘必须保持密封吸附
（ATTACHED 控制环照跑，互锁/漏气监护对它照常成立）。

### 3.4 交接运动（update() 新 4.7 步，紧跟 4.6 倾身之后）

与 4.5（下滑补偿）/4.6（倾身）同一类"直接改足端目标"的全局步：

```python
# 4.7 零力交接：被抬腿沿上坡还 δ（卸载），其余支撑腿各沿下坡 δ/n（接载），
# 均值不变=身体指令不动；按真实时间匀速铺（载荷重分配是准静态过程），
# 漏气挽救期暂停（漏着的盘摩擦余量低，不该被推）。铺完才放气进 VENT。
if (not leak_pause and self._slot_active and self._slot_leg is not None
        and self.phase_of[self._slot_leg] == LegPhase.HANDOVER):
    cur = self._slot_leg
    step = min(self._ho_left, HANDOVER_SPEED_MMS * dt)
    self._ho_left -= step
    dx, dy = self._down
    sup = [n for n in LEG_NAMES if self.phase_of[n] == LegPhase.STANCE]
    self.foot[cur][0] -= dx * step          # 反下坡 = 上坡 = 卸载方向
    self.foot[cur][1] -= dy * step
    for n in sup:                           # 用实际支撑数而不是写死 5
        self.foot[n][0] += dx * step / len(sup)
        self.foot[n][1] += dy * step / len(sup)
    if self._ho_left <= _EPS:
        self.ctl.request_release(LEG_NAMES.index(cur))
        self.phase_of[cur] = LegPhase.VENT
```

`_step_swing` 加一个 `elif ph == LegPhase.HANDOVER: pass` 分支（运动在 4.7
驱动；`_seg_t` 照常计时备诊断）。不需要独立超时监护：速率固定必然铺完，
唯一能拖住它的是 leak_pause，而漏气自己有 `leak_rescue_s` 冻结兜底。

### 3.5 引擎新状态

`__init__` 加 `self._ho_left = 0.0`。一次一腿的窗序保证同时至多一个交接
在途，不需要按腿存。

## 4. 与现有机制的相互作用（逐条，全是坑）

1. **⚠ 支撑场跟随（最大的坑）**：update() 第 4 步现在只让
   `(STANCE, VENT)` 的脚随支撑场平移并保持压入深度。HANDOVER 期间脚仍然
   吸在墙上，行走模式下身体在动，**必须一并跟随**，否则该脚在墙面系被拖着
   划、蹭移还密封着的唇口。改为 `(STANCE, VENT, HANDOVER)`，z 保持
   压入深度的那行同样覆盖 HANDOVER。交接位移（4.7）叠加在场平移之上，
   与 sag_comp 叠加口径相同。
2. **相位钟**：HANDOVER 非 STANCE，第 3 步走 `adv = min(dt, 窗剩余)` 分支
   ——爬行模式窗头 0.6s 用不完 1.7s 交接，钟停在窗尾等它，周期自然拉宽
   （与现有"钟停等吸附事件"同语义，不需要新代码）。原地抬起（零速）钟本来
   就空转，无影响。
3. **sag_comp（4.5）**：注入条件是 slot 腿在 LIFT/TRANSFER，交接期间不注入,
   天然互斥。**交接生效后建议把 sag_comp 归零停用**——它是"坠后追补"的
   治标层，两者同开会双份推挤支撑系白吃工作空间（A/B 期间只开一个）。
4. **倾身（4.6）**：lean 只在全腿 STANCE/HOVER 时推进，HANDOVER 自动把它
   暂停——不改代码，行为正确（交接期间不该再叠加整体平移）。
5. **漏气**：4.7 有 `not leak_pause` 门——交接暂停不丢，与 4.5/4.6 同禁区;
   挽救超时走现有冻结。
6. **冻结/解冻**：frozen 时 update() 顶部早退，交接自动保持（目标全冻）。
   `clear_freeze()` **不取消**在途交接（与"摆动中的 step_active 保留"同
   口径：这条腿已经过了窗头决策必须走完收口；且交接是慢速可逆动作，恢复
   续铺无害）。注意与"未铺完倾身取消"的差别：倾身是纯人工请求可弃，交接
   是抬腿动作的一部分。
7. **工作空间账**：支撑腿因交接的最大额外下坡外摆 ≈ δ（一个周期内攒 5 次
   δ/5，轮到自己抬时才抵回）。
   - body_lean 原地实验（零速、站位、stand 62）：余量 ≥80mm，δ≤20 随便放,
     不加码。
   - climb_walk 行走：δ 要进支撑尾总账（现账 = 半步幅 + 5×sag_comp + VENT
     随场拖尾，见 `comp_tail`）。v1 先文档声明"大步幅 + 大 δ 不同开"，
     启动打印把 δ_max 与 `comp_tail − 半步幅` 并排亮出来；把 δ 计进
     `max_straight_step` 与 sag_comp 动态限额的公式留到交接在实验里站住后
     再做（避免为还没标定的参数改总账公式）。
   - 被抬腿自己的 +δ 上坡方向是它即将摆去的方向，不吃账。
8. **互锁/门槛**：窗头决策已检查过（其余 5 足 ATTACHED 且盘压深于
   lift_gate_kpa）才进 HANDOVER；交接 ≤2s 内若支撑腿翻脸走漏气路径（见 5）。
   交接完成时不重查互锁——与现行"VENT 起不再重查"同口径。
9. **request_lift / 单步 / 连续行走**：三条路都走同一个窗头决策，交接
   自动全覆盖；`air_mode` 吸不上照走交接（无害，多花时间），不加旁路。
10. **启动序列 / RETRY_LIFT**：启动逐足压入无抬腿，重试回抬时吸盘本来就是
    FAULT 未吸附——都无储能可释放，不经过 HANDOVER（代码路径天然不经过，
    确认即可）。
11. **遥测**：`PHASE_CH` 加 `"handover": "Z"`（§3.2）；黑匣子相位跳变自动
    记录。可选：TLM tag 里加剩余交接量（`交余=X.X`），实现时看着办。

## 5. δ 标定

起标值 = 08-20 实测单次弹跳 ×0.8（宁欠勿过纪律，与 sag_comp 同）：

| 腿 | L1 | R1 | L3 | R3 | R2 | L2 |
|----|----|----|----|----|----|----|
| 实测弹跳 mm | 20.9 | 19.2 | 13.5 | 11.8 | 5.9 | 5.8 |
| **δ 起标 mm** | **17** | **15** | **11** | **9** | **5** | **5** |

- 这组数是**原地踏步、stand 62、trim 6** 工况的稳态值；行走模式支撑场
  持续绕紧，前腿真实储能可能更大——先用 body_lean 原地 A/B 标定，
  再上 climb_walk 单步（i 键）微调，最后连续行走。
- 过冲的代价：δ 超过真实储能会把被抬腿反向预载（释放时身体向上弹一下）
  ——小过冲无害甚至顺向，大过冲白吃工作空间 + 反向蹬一脚，×0.8 起步逐次加。
- v2 方向（本期不做）：交接过程中盯整机电流（实验显示 0.73→1.9A 的信号
  量级足够，0.5s 采样可用）——电流降到平台 ≈ 卸载到位，可做成自适应 δ。

## 6. 脚本接口

两个脚本都加 `--handover` 参数，格式二选一：

- `--handover 8`：六腿统一 δ=8；
- `--handover L1:17,R1:15,L3:11,R3:9,R2:5,L2:5`：逐腿设（可只给部分腿，
  未给的保持 0）。

解析后 `replace(leg, handover_mm=…)` 进 cfg（照抄 `--press-delta` 的
replace 模式）。范围校验 0~25mm（>21 实测封顶没意义）。启动打印每腿 δ 与
交接段时长。body_lean 文档头/键位说明补一句"抬起前自动先做零力交接
（--handover 开启时）"。climb_walk 同步。

## 7. 测试清单（tests/test_climb.py，照现有风格）

1. **回归**：`handover_mm=0`（默认 CFG）行为与现在逐字节一致——现有 73 测
   全绿本身就是断言，不用新写。
2. **相位序列**：δ>0 时窗头决策后进 HANDOVER，期间 `ctl.state` 保持
   ATTACHED、阀不动；δ/HANDOVER_SPEED_MMS 秒（±3 DT）后才 request_release
   进 VENT（照抄 test_vent_before_lift 的结构）。
3. **位移账**：交接完成时被抬腿目标 = 起点 + 上坡 δ，每条支撑腿 = 起点 +
   下坡 δ/5，y 分量按 _down 口径、z 恒在压入位；六腿位移矢量和 = 0
   （均值不变=身体指令不动的代数断言）。
4. **行走叠加**：带速度走，HANDOVER 期间被抬腿跟随支撑场（照抄
   test_vent_before_lift 的 XY 随场断言，叠加卸载分量后的期望值）。
5. **漏气暂停**：交接中注入支撑腿漏气，_ho_left 冻结不推进、不放气；
   挽救成功后续铺完成。
6. **冻结恢复**：交接中 frozen 注入，目标保持；clear_freeze 后续铺、
   完成、正常走完抬落（对比 test_single_step_hover_survives_freeze）。
7. **航向旋转**：wz 转过 θ 后交接方向随 _down 旋转（照抄
   test_sag_comp_downhill_rotates_with_heading）。
8. **原地抬落全程 IK 可解**：request_lift + δ=20 极限值，全程 pulses 可解、
   支撑足除交接分量外纹丝不动（对比 test_lift_in_place_hover_and_land_back
   ——注意该测试现有"支撑足纹丝不动"断言在 δ>0 时要改成"只动交接分量"）。
9. **δ 解析**：脚本层参数两种格式 + 越界拒绝（如果解析函数放引擎外，
   测试放脚本侧或直接单测解析函数）。

## 8. 实现清单（逐文件）

| 文件 | 改动 |
|------|------|
| `hexapod/config.py` | LegConfig.handover_mm 字段 + 注释（引本文档与报告） |
| `hexapod/climb.py` | HANDOVER_SPEED_MMS 常量；LegPhase.HANDOVER；`__init__` _ho_left；窗头决策分叉（§3.3）；第 4 步跟随集合加 HANDOVER（§4.1）；新 4.7 步（§3.4）；_step_swing pass 分支；模块 docstring 补 HANDOVER 段说明 |
| `hexapod/runlog.py` | PHASE_CH 加 "handover": "Z" |
| `scripts/body_lean.py` | --handover 解析 + 启动打印 + 文档头 |
| `scripts/climb_walk.py` | 同上（两边同步条款照旧） |
| `tests/test_climb.py` | §7 的 8 项 |
| `docs/P4-GUIDE.md` | 排障表"墙上原地踏步"行补零力交接入口（可选） |

## 9. 验收（实现完成的定义）

1. 全测试绿（现有 73 + 新增）。
2. mock 冒烟：body_lean --mock 带 --handover 走完整抬落轮。
3. **实机 A/B（下下步）**：重跑 08-20 原地踏步实验（同参数 + --handover
   起标表）——每轮下滑 74mm 应降一个数量级（<10mm）；整机电流不再逐周期
   爬升（保持 ~0.7-1.0A）；视频里 vent 无可见弹跳。量化管线复用
   （NCC 追踪，见报告"量化管线"脚注）。
4. A/B 过了再上 climb_walk：先单步、后连续，δ 微调，然后重测净前进率
   （26% 基线）。

## 附录 A：1-D 弹簧模型（验证脚本，可直接跑）

```python
def sim(mode, cycles=300, u=40.0, mg=89.0):
    """y_i=锚点-指令; X=mean(y)-mg/n; 吸附零预载锁定 y_new=X。
    mg/k=89mm 由原地实测 74mm/轮反推。u=步幅(mm), 指令净前进=6u/5/周期。"""
    y = [0.0] * 6
    xs = []
    for c in range(cycles):
        for j in range(6):
            att = [i for i in range(6) if i != j]
            if mode == "handover":          # vent 前零力交接
                x6 = sum(y) / 6 - mg / 6
                fj = y[j] - x6
                y[j] -= fj
                for i in att:
                    y[i] += fj / 5
            for i in att:                   # 窗内支撑场扫 u/5
                y[i] += u / 5
            y[j] = sum(y[i] for i in att) / 5 - mg / 5   # 重吸附锁定
        xs.append(sum(y) / 6 - mg / 6)
    return xs[-1] - xs[-2]                  # 稳态净前进/周期

# 基线 u=40: 12.4mm (25.8%, 实测 26%); handover: 48.0 (100%)
# 基线 u=0: -35.6mm/轮 (实测 -74, 模型半量级——真机还有姿态非线性); handover: 0
```

模型已知的偏差：真机 lean 均匀指令亏 60%（线性模型预测 0%）——存在姿态
相关非线性损耗层，交接治不了它，靠标定/机械加刚；单腿储能有扭矩饱和上限
（~21mm），模型无此封顶。两条都不影响交接机构本身的正确性。
