#!/usr/bin/env python3
"""地-墙过渡首批实验（P5 探索线，2026-09-06）：前足上墙 / 身体俯仰 / 收腿静停。

引擎 hexapod/mount.py（MountEngine）：每腿一个接触面（地面/墙面/空中），
挪腿沿各自面的法向抬离与压入，身体位姿（俯仰、离墙距离、高度）慢速改变时
接触足在世界系钉死。吸附状态机与黑匣子与 climb_walk 同源。

场地：竖直玻璃墙 + 墙脚地面铺玻璃板（六足都能吸附——四足阶段地面要靠后足
吸附提供推力，tools/mount_analysis.py 四节：换前足瞬间单墙盘剪切上界 20~23N
超过实测 15N，地面不提供推力就悬）。机器人头朝墙、正对墙放在玻璃板上，
用卷尺量前腿 coxa 舵机轴到玻璃面的水平距离填 --wall-dist（默认 140）。
全程安全绳，人在旁。

流程与 body_lean 同口径：缓慢站起（爬墙站位）-> 就位暂停（量距、p）-> 六足
逐足压入吸附 -> 键盘实验。
  就位暂停时：卷尺量前腿 coxa 舵机轴到玻璃面的水平距离（站起后才准——上电
  瞬间机身可能挪几毫米），与 --wall-dist 不符就按 d 输入实测值回车，引擎把
  参考位置改过来并重算可落足带，再按 p。启动后不能再改
  1~6  选腿（1=L1 2=L2 3=L3 4=R1 5=R2 6=R3）
  w    选中腿→墙面：抬起、coxa 摆到指正前（β=0，前腿 -55°）、弧线平移到墙前
       15mm 悬停（落点高 --wall-height，缺省取当前位姿可落足带中点；带由
       引擎按倾角≤12°+IK 余量+压深实算并打印）。悬停时先目视吸盘对不对正、
       离墙多远，再按 i 沿墙法向（+x）压入 press_delta 抽气确认
       ⚠ **前足这个落点高度就是后面的抬头预算**：落点每高 1mm 约多 0.45° 抬头，
       落地接管每 1mm 抵掉同样多（两者花的是吸盘倾角那同一笔 15°）。缺省的带
       中点是照顾落点密封的选法，比"轴⊥墙"的高度还低几毫米——09-12 实机就是
       这样把抬头从 14° 丢到 6°（docs/WALL-MOUNT-LAB.md §3）。按 w 时屏幕会把
       ⊥ 点、此落点/带上沿各能抬到几度一起打出来，要抬头就显式给 --wall-height
  g    选中腿→地面回位（该腿爬墙站位在当前位姿下投影到地面的点）
  b    选中腿→地面正后方 --rear-dist（后腿 coxa 后摆指正后，β=180°）
  t    选中腿绕髋**摆**到地面站位前方 --fwd-dist（默认 85），髋足距离不变、吸盘
       轴仍⊥地面：做前足上墙**之前**先把两条中腿摆到前髋底下。默认站位的中足在
       机身中心正下方，与重心几乎重合，抬起第二只前足时前半机身成悬臂——09-09
       实测前缘下沉 26mm 且吸住后不回弹（L1 那次只沉 6mm，因为当时中腿还兜得住）。
       ⚠ 是摆不是平移：平移会把腿伸长（中腿前移 85 时髋足距 176.6→196.0），倾角
       涨到 10.6°，虽在 12° 带内但目视明显斜（09-09 实机）；摆动时倾角恒 0
       --fwd-reach R：摆的同时把髋足水平距离收到 R（09-13 用户：中腿收近承重更省力；
       机身没升高时吸盘在腿平面内会斜，只承重腿 35° 以内放行）
  h    选中腿收起悬空（抬 15mm→缩到髋外 0.6 站位半径、站位面上 45mm，留在
       空中随身体动；不承载，互锁不算它）
  z    把载荷**接管**到选中的这条接触腿身上（--takeover-step，默认 3mm，可连按）：
       该腿足端指令沿世界竖直压下 3mm、其余接触腿各松 3/n，六腿偏移和仍为 0 ⇒
       身体指令不动，但载荷（=其余腿攒着的弹性势能）转到这条腿。L1 抬上墙吸住
       后尤其该做：那时 L1 是零预载的新腿，机身重量还压在被压弯的地面腿里，
       转过去就等于就地把势能卸掉，下一条腿（R1）再抬时储能小、下沉也小。
       盯着盘压/电流/机身高度一次 3mm 地加；墙面腿吃的是剪切（实测容量 15N），
       地面腿吃的是压深（余量 = 28−press_delta，默认只有 10mm）
  i    悬停腿落下压入吸附（DESCEND→PRESS→WAIT；FAULT 加深重试、耗尽冻结）
       抬离高度分面：墙面 15mm（吸盘回弹口径，也是悬停目测基准），地面
       --floor-clear（默认 45）——地面上腿一抬因自重下垂十几到二十几毫米，
       抬 15 会让盘在摆动全程蹭地板（09-09 实机 L2/R2 摆动碰玻璃）
  . ,  墙面目标修正 ±2mm（. 往墙里补、, 退回，范围 -20~+40），**只改当前那条腿**
       （有悬停腿就改它，否则改 1~6 选中的腿）：悬停时目测吸盘离玻璃不是 15mm
       就按这个补到 15 再按 i。修正量逐腿独立，作用于该腿的墙面目标（悬停点、
       压入位、落点带），地面目标不动；每条腿标出来的值记下来，下次
       --wall-trim L1:16,R1:0 直接给。⚠ 修正量直接叠进压入深度：给大了等于命令
       腿往刚性玻璃里多压这么多，吸不上还会自动加深——宁可先给小的
       （09-09 实机：wall_dist 155 修正 16 时 L1 停在玻璃外 15mm 而 R1 已贴上，
       两只前腿差 16mm，逐腿标才对）
  +/-  悬停中把这条腿的落点沿墙上下挪 ±5mm（= 同 +），按 20mm/s 匀速铺过去
       （不跳变；铺完才让按 i）：抬一条前腿机身前部就被腿链弹性压沉一截
       （08-19 实测 13~27mm），抬第二条再沉一截，模型以为机身还在指令位姿上，
       按同一世界高度放 R1 就会比 L1 低、还会更贴墙——悬停时目测两盘高度，用这个
       把后放的一只提到与先放的一只齐平。落点带与压深会复核，出带会拒。
       ⚠ 顺序：盘要是已经贴到玻璃（下沉所致），先用 , 退开再调高度——贴着调
       等于蹭着玻璃走；调完再用 . 把间距压回 15
  ↑/↓  俯仰 ±--pitch-step（抬头为正，上限 --pitch-max）
  ←/→  身体离墙/贴墙 5mm      [/]  身体降/升 5mm
       位姿改变按 2°/s、10mm/s 铺设；整段中间位姿逐个预检（接触足倾角≤15°、
       IK 余量、膝/机身腹面不撞地不撞墙），任一不过整段拒绝、原地不动
  空格 取消位姿铺设（停在当前位姿）   f 解冻   o×2 取机   ESC×2 退出

零力交接（--handover δ，默认关）：w/g/b/t/h 抬腿之前，先把这条腿的足端指令沿
世界竖直还 δ（把它的力卸到 ~0）、其余接触腿各压 δ/n 接住载荷，六腿偏移代数和
恒为 0 ⇒ 身体指令不动；铺完（δ/rate 秒）才放气。08-20 量化：83% 的下滑发生在
放气密封破裂一瞬——被抬腿吸附期间攒的弹性势能一步释放；09-09 在过渡实验上
复现为"抬一只前足机身沉 26mm 且吸住后不回弹"。落地吸住后自动反向做一次
（接管）把借来的份额还回去，不还的话份额会一次次往下累积把压深吃穿。
⚠ 方向是竖直不是面法向：地面足竖直=法向（等价于改压深），墙面足竖直=切向
（沿墙剪切，往墙里压根本接不了体重）。⚠ 地面足的卸载方向就是把盘往上拔的
方向——δ 给过头会把盘直接拔下来（表现为盘压回升→漏气挽救），宁欠勿过。

首批实验（依据 tools/mount_analysis.py，09-06 评估）：
  E1  地面六足吸住 → 1 w 悬停看对正 → i 吸附 → g 回地。验证：coxa 内摆 55°
      不撞、压入方向改为 +x 后的吸附确认、可落足带与实机对得上
  E2  1 w i、4 w i（双前足上墙）→ ↑ 若干档到 10~15° → 手机侧拍量真实俯仰
      与指令之比（腿链弹性在这个构型下是多少现在完全不知道）
  E3  E2 之后 2 h、5 h（中腿收起，四接触）→ 静停 30s 看盘压/电流 → 1 h
      （抬一只前足，只剩一墙盘）看另一盘扛不扛得住——最大风险项，低俯仰、
      安全绳绷紧再做

用法:
  python mount_wall.py --mock                  # 无硬件干跑
  python mount_wall.py --dry                   # 真舵机 + 仿真气路（吸附确认是假的）；
                                             # 真阀只在需要时通电排气：站起时六阀排气、
                                             # 站定后全断；哪条腿要抬（进 VENT 相位）
                                             # 就只给那一路通电，落回支撑后断电——
                                             # 09-08 实测：阀不通电时站起一压，吸盘经
                                             # 单向阀被挤成被动真空锁脚；六阀长通又
                                             # 发热严重（≈25W）
  python mount_wall.py --no-tank --wall-dist 140
  python mount_wall.py --no-tank --wall-dist 140 --wall-height 230 --pitch-step 3
  # 治下沉：开零力交接 + L1 上墙后主动接管（起标值宁欠勿过，逐次加）
  python mount_wall.py --no-tank --wall-dist 155 --support-legs L2,R2 \
      --wall-trim L1:16,R1:0 --handover L1:10,R1:10,L2:5,R2:5,L3:8,R3:8 \
      --takeover L1:10,R1:10
  善后（放气+回地面站姿）：python climb_walk.py --release

黑匣子：software/logs/mount_YYYYmmdd_HHMMSS.log（与 climb_walk 同机制；
位姿变化、挪腿事件、拒绝原因全部落盘）。安全口径与 body_lean 完全一致
（ESC×2 序列屏蔽 Ctrl-C、IO/工作空间双降落伞、欠压立即停机）；退出/取机
序列从 body_lean 复制——改那边记得同步改这边。
"""
import argparse
import os
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver
from hexapod.adhesion import (AdhesionController, MockVacuumIO, FootState,
                              GroundVent, Pi5VacuumIO,
                              ATTACH_KPA, PUMP_ON_KPA, PUMP_OFF_KPA)
from hexapod.climb import parse_leg_order, parse_handover, PRESS_DEPTH_MAX
from hexapod.mount import (MountEngine, MountPhase, FLOOR, PITCH_RATE_DPS,
                           LIN_RATE_MMS, COXA_MAX_DEG, BELLY_MM, VALVE_OPEN_PHASES,
                           FLOOR_CLEAR_MM, TAKEOVER_STEP_MM, TAKEOVER_MAX_MM,
                           SUPPORT_TILT_DEG, SLIDE_UNLOAD_MM)
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES
from hexapod.kinematics import WorkspaceError
from hexapod.runlog import RunLog, ClimbWatch
from hexapod.powerlog import (PowerWatch, startup_marker, servo_power_on,
                              servo_relay_close)

from climb_walk import status_line, coils_off   # 显示/收尾与 climb_walk 同源

STATUS_S = 0.5
LEG_KEYS = {"1": "L1", "2": "L2", "3": "L3", "4": "R1", "5": "R2", "6": "R3"}
LIN_STEP_MM = 5.0
ARROWS = {b"[A": "UP", b"[B": "DOWN", b"[C": "RIGHT", b"[D": "LEFT"}


def parse_per_leg(spec, flag, lo, hi, what="值", sample="L1:16,R1:0"):
    """逐腿参数解析：'16'=全腿统一，'L1:16,R1:0'=逐腿（未给的腿 0）。
    返回 {腿名: mm}；非法抛 ValueError（脚本层转 ap.error）。"""
    spec = spec.strip()
    if not spec:
        raise ValueError(f"{flag} 不能为空")
    def _v(t):
        try:
            v = float(t)
        except ValueError:
            raise ValueError(f"{flag} {what} {t!r} 不是数字")
        if not lo <= v <= hi:        # nan 比较为假一并拒
            raise ValueError(f"{flag} {v:g} 非法：范围 {lo:g}~{hi:g}mm")
        return v
    if ":" not in spec:
        return {n: _v(spec) for n in LEG_NAMES}
    out = {}
    for part in spec.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        if ":" not in part:
            raise ValueError(f"{flag} 逐腿格式为 {sample}，看到 {part!r}")
        name, _, val = part.partition(":")
        name = name.strip().upper()
        if name not in LEG_NAMES:
            raise ValueError(f"{flag} 未知腿名 {name!r}（可选 {'/'.join(LEG_NAMES)}）")
        if name in out:
            raise ValueError(f"{flag} 腿 {name} 给了两次")
        out[name] = _v(val.strip())
    if not out:
        raise ValueError(f"{flag} 没解析出任何腿")
    return out


def parse_wall_trim(spec):
    """--wall-trim：墙面目标修正 mm，逐腿。"""
    return parse_per_leg(spec, "--wall-trim", -20.0, 40.0, "修正量")


def parse_takeover(spec):
    """--takeover：新腿落地吸住后主动接管的载荷 mm，逐腿。"""
    return parse_per_leg(spec, "--takeover", 0.0, TAKEOVER_MAX_MM, "接管量",
                         "L1:12,R1:12")


def read_key(timeout):
    """body_lean.read_key 的四方向变体：↑/↓/←/→ 识别为 UP/DOWN/LEFT/RIGHT，
    其余转义序列整包丢弃，裸 ESC 语义不变（退出确认键）。"""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    data = os.read(sys.stdin.fileno(), 8)
    if not data:
        return None
    if data[0:1] == b"\x1b":
        if len(data) > 1:
            return ARROWS.get(data[1:3])
        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
        if r2:
            tail = os.read(sys.stdin.fileno(), 8)
            return ARROWS.get(tail[:2])
    return data[:1].decode("latin-1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--dry", action="store_true",
                    help="真舵机 + 仿真气路：吸附确认是假的，纯排练动作；真阀按需排气："
                         "站起时六阀通电排气、站定后全断，抬腿前只给那一路通电、落回"
                         "支撑后断电（否则站起一压被被动真空锁脚；六阀长通又发热，"
                         "09-08 实测）；泵不动，退出断线圈")
    ap.add_argument("--no-tank", action="store_true",
                    help="无罐：泵直抽歧管；没有储备真空（地面/上墙均已多次实测可用）")
    ap.add_argument("--wall-dist", type=float, default=140.0,
                    help="起始时前腿 coxa 舵机轴到墙面的水平距离 mm（卷尺量，"
                         "默认 %(default)g，范围 100~220）。前腿可落足带随它变，"
                         "引擎按实际几何算")
    ap.add_argument("--wall-height", type=float, default=None,
                    help="前足上墙落点离地高度 mm（⚠ 这个数就是后面的抬头预算："
                         "每高 1mm 约多 0.45° 抬头，见文件头 w 键；"
                         "默认取当前位姿可落足带中点；"
                         "范围 100~500，出带引擎拒绝并打印带）")
    ap.add_argument("--rear-dist", type=float, default=None,
                    help="b 键：后足落到髋正后方多远的地面 mm（默认=该腿爬墙站位"
                         "半径 ≈176，吸盘轴 ⊥ 地面；范围 120~220，更近会带面内倾角）")
    ap.add_argument("--attach-order", default=None,
                    help="启动逐足压入次序，六腿排列如 R3_R2_L3_R1_L2_L1（下划线或逗号"
                         "分隔，不分大小写，不缺不重）。默认=窗序 R3_L1_R2_L3_R1_L2。"
                         "诊断用：吸不上的腿排到最后，其余五足吸牢当反力座再压它——"
                         "启动早段只有一两足吸住时，压入反力会把机身顶起而不是把盘"
                         "压进地面")
    ap.add_argument("--wall-trim", default=None,
                    help="墙面目标修正 mm（正=再往墙里压，范围 -20~40）：统一值如 16，"
                         "或逐腿 L1:16,R1:0（未给的腿 0）。上次实验用 . , 各腿补到目测"
                         "15mm 的量。只作用于墙面目标，且叠进压入深度——给大了=命令腿"
                         "往刚性玻璃里硬压，宁可给小的")
    ap.add_argument("--support-legs", default=None,
                    help="只承重不吸附的腿，如 L2,R2：压到位即回支撑、不抽气，也不参与"
                         "互锁与漏气监护；抬腿前照样开阀放气 lift_vent_s 再抬（气路接着"
                         "会被单向阀憋出被动真空，09-13）。用于 09-09 实机情形——上墙"
                         "过程中中腿吸盘吸不住地面，但压着能靠摩擦当支撑；不设的话"
                         "互锁会因'L2 未吸附'拒绝一切动作。六条全写 = 泵不开的摩擦上墙"
                         "前期（docs/WALL-MOUNT-FRICTION.md），要配 --dry：实机模式下"
                         "只承重腿的阀一直通电排气，六路长通发热。⚠ 这些腿只能承压不能"
                         "承拉，身体俯仰后扛不住剥离力矩，别把它们算成安全余量")
    ap.add_argument("--slide-legs", default=None,
                    help="随动腿（只承重的地面腿，如 L2,R2）：抬头/升降/平移机身时贴地跟着滑，保持"
                         "相对机身的姿势——coxa 角不变，髋足距离取吸盘在腿平面内最正的值（机身升高时"
                         "自己往里收）。每段先开阀放气、少压 --slide-unload 再滑，铺完压回（09-13 用户："
                         "中腿同步微调，滑前先通电磁阀）。必须也写在 --support-legs 里")
    ap.add_argument("--slide-unload", type=float, default=SLIDE_UNLOAD_MM,
                    help="随动腿滑之前少压的量 mm（默认 %(default)g，范围 0~10）")
    ap.add_argument("--floor-clear", type=float, default=FLOOR_CLEAR_MM,
                    help="地面移动的抬离高度 mm（默认 %(default)g，范围 20~120）：墙面用"
                         f"的 {DEFAULT_CONFIG.lift_clearance:g} 是按吸盘回弹定的，地面上"
                         "腿一抬还会因自重下垂十几到二十几毫米，抬少了盘在摆动全程蹭"
                         "地板（09-09 实机 L2/R2 摆动碰玻璃）。墙面抬离量不受影响。"
                         "抬头后机身比模型低得更多（09-13 摩擦上墙实机：指令 16° 时约低 40mm），"
                         "45 在指令 24° 挪中腿时蹭地，摩擦上墙前期用 100")
    ap.add_argument("--fwd-dist", type=float, default=85.0,
                    help="t 键：该腿绕髋摆到地面站位前方多远 mm（默认 %(default)g，"
                         "范围 0~160）。中腿 85 = 摆到前髋正下方（coxa 偏 29°，"
                         "髋足距离与吸盘垂直度都不变）")
    ap.add_argument("--fwd-reach", type=float, default=None,
                    help="t 键：脚离髋的水平距离 mm（默认=站位半径 ≈177，平身时吸盘轴 ⊥ 地面；"
                         "范围 100~200）。给小了 = 把脚往身体收（09-13 用户：中腿收近承重更省力"
                         "——指令 22° 时从 177 收到 140，femur 每 10N 负载的扭矩 12.3→8.6kg·cm），"
                         "代价是机身没升高时吸盘在腿平面内会斜（140 时总倾角约 30°，只承重腿 "
                         "35° 以内放行）；机身升高后垂直解本来就往里收（髋高 160→161mm、180→140mm）")
    ap.add_argument("--handover", default=None,
                    help="零力交接量 δ mm（抬腿前先把这条腿的力卸到零）：统一值如 12，"
                         "或逐腿 L1:17,R1:15（未给的腿 0=关，默认全关）。抬腿前该腿"
                         "足端指令**竖直**抬 δ、其余接触腿各压 δ/n，六腿偏移和为 0 ⇒ "
                         "身体指令不动，铺完才放气——治 09-09 实测'抬一只前足机身沉 "
                         "26mm 且不回弹'（08-20 量化：83%% 的下滑发生在放气密封破裂一"
                         "瞬）。⚠ 地面足的卸载方向就是把盘往上拔的方向，δ 给过头会把"
                         "盘直接拔下来（墙面足是沿墙剪切，过头只是反向预载）——宁欠"
                         "勿过，从实测弹跳×0.8 起标、逐次加。范围 0~45")
    ap.add_argument("--handover-rate", type=float, default=None,
                    help=f"交接/接管的铺设速率 mm/s（默认 "
                         f"{DEFAULT_CONFIG.handover_rate_mms:g}，范围 0~50）："
                         "吸住的脚改指令=改力，载荷重分配要留准静态时间")
    ap.add_argument("--takeover", default=None,
                    help="新腿落地吸住后主动接管的载荷 mm（逐腿，如 L1:12 或统一值）："
                         "L1 抬上墙吸住时它是零预载的新腿，机身重量还压在被压弯的地面"
                         "腿里——把载荷转到它身上=就地卸掉那些腿攒的弹性势能，下一条腿"
                         "（R1）再抬时储能就小了。不给也会自动把抬它时借走的份额还回去"
                         f"（接管量取 max(欠账, 本值)）。范围 0~{TAKEOVER_MAX_MM:g}；"
                         "⚠ 墙面腿接的是剪切（实测容量 15N），逐次加、盯盘压电流")
    ap.add_argument("--support-tilt", type=float, default=SUPPORT_TILT_DEG,
                    help="只承重腿（--support-legs）的吸盘倾角容差°（默认 %(default)g，"
                         "范围 0~60）：它只压不吸，盘面对不对正不影响传正压力，所以不该"
                         "用吸附腿那条 15°（密封容差）卡它。09-12 台架实测盘面斜 35° 时"
                         "接触仍在盘面、仍压得住；35° 也正是中腿在俯仰 90° 时盘面最多"
                         "斜到的角度，所以默认值等于'倾角这条永远不卡中腿'")
    ap.add_argument("--takeover-step", type=float, default=TAKEOVER_STEP_MM,
                    help="z 键每按一次手动接管的量 mm（默认 %(default)g，范围 1~10）")
    ap.add_argument("--pitch-step", type=float, default=5.0,
                    help="每按一次 ↑/↓ 的俯仰量°（默认 %(default)g，范围 1~10）")
    ap.add_argument("--pitch-max", type=float, default=30.0,
                    help="俯仰上限°（默认 %(default)g，范围 0~90）：首批实验只到"
                         " 30，四足扶梯阶段另开")
    ap.add_argument("--press-delta", type=float, default=None,
                    help="预压行程 mm，覆盖全部腿（默认用 config 值 "
                         f"{DEFAULT_CONFIG.legs[0].press_delta_mm:g}）")
    ap.add_argument("--stand-height", type=float,
                    default=DEFAULT_CONFIG.stand_height,
                    help="站高 mm（默认 %(default)g，范围 55~95）")
    ap.add_argument("--tilt-trim", type=float,
                    default=DEFAULT_CONFIG.cup_tilt_trim_deg,
                    help="吸盘轴垂直度实测修正角°（默认 %(default)g，±8）")
    ap.add_argument("--startup-gap", type=float, default=0.0,
                    help="六阀线圈通电完毕到舵机继电器合闸之间静置秒数（默认 0）")
    ap.add_argument("--relay-first", action="store_true",
                    help="舵机继电器合闸前置（串口一开就合 GPIO17，阀线圈通电后才"
                         "固件使能；09-07 启动死机止血口径）")
    args = ap.parse_args()
    if not 100.0 <= args.wall_dist <= 220.0:
        ap.error(f"--wall-dist {args.wall_dist:g} 非法：范围 100~220mm")
    if args.wall_height is not None and not 100.0 <= args.wall_height <= 500.0:
        ap.error(f"--wall-height {args.wall_height:g} 非法：范围 100~500mm")
    if args.rear_dist is not None and not 120.0 <= args.rear_dist <= 220.0:
        ap.error(f"--rear-dist {args.rear_dist:g} 非法：范围 120~220mm")
    attach_order = None
    if args.attach_order is not None:
        try:
            attach_order = parse_leg_order(args.attach_order)
        except ValueError as e:
            ap.error(str(e).replace("--leg-order", "--attach-order"))
    wall_trim = {}
    if args.wall_trim is not None:
        try:
            wall_trim = parse_wall_trim(args.wall_trim)
        except ValueError as e:
            ap.error(str(e))
    support_only = ()
    if args.support_legs:
        support_only = tuple(t.strip().upper()
                             for t in args.support_legs.replace("_", ",").split(",")
                             if t.strip())
        bad = [n for n in support_only if n not in LEG_NAMES]
        if bad:
            ap.error(f"--support-legs 未知腿名 {bad}（可选 {'/'.join(LEG_NAMES)}）")
    slide_legs = ()
    if args.slide_legs:
        slide_legs = tuple(t.strip().upper()
                           for t in args.slide_legs.replace("_", ",").split(",") if t.strip())
        extra = [n for n in slide_legs if n not in support_only]
        if extra:
            ap.error(f"--slide-legs {extra} 不在 --support-legs 里：吸住的脚滑不了")
    if not 0.0 <= args.slide_unload <= 10.0:
        ap.error(f"--slide-unload {args.slide_unload:g} 非法：范围 0~10mm")
    if not 20.0 <= args.floor_clear <= 120.0:
        ap.error(f"--floor-clear {args.floor_clear:g} 非法：范围 20~120mm")
    if not 0.0 <= args.fwd_dist <= 160.0:
        ap.error(f"--fwd-dist {args.fwd_dist:g} 非法：范围 0~160mm")
    if args.fwd_reach is not None and not 100.0 <= args.fwd_reach <= 200.0:
        ap.error(f"--fwd-reach {args.fwd_reach:g} 非法：范围 100~200mm")
    handover = {}
    if args.handover is not None:
        try:
            handover = parse_handover(args.handover)
        except ValueError as e:
            ap.error(str(e))
    takeover = {}
    if args.takeover is not None:
        try:
            takeover = parse_takeover(args.takeover)
        except ValueError as e:
            ap.error(str(e))
    if args.handover_rate is not None and not 0.0 < args.handover_rate <= 50.0:
        ap.error(f"--handover-rate {args.handover_rate:g} 非法：范围 0~50mm/s")
    if not 0.0 < args.support_tilt <= 60.0:
        ap.error(f"--support-tilt {args.support_tilt:g} 非法：范围 0~60°")
    if not 1.0 <= args.takeover_step <= 10.0:
        ap.error(f"--takeover-step {args.takeover_step:g} 非法：范围 1~10mm")
    if not 1.0 <= args.pitch_step <= 10.0:
        ap.error(f"--pitch-step {args.pitch_step:g} 非法：范围 1~10°")
    if not 0.0 <= args.pitch_max <= 90.0:
        ap.error(f"--pitch-max {args.pitch_max:g} 非法：范围 0~90°")
    if not 55.0 <= args.stand_height <= 95.0:
        ap.error(f"--stand-height {args.stand_height} 非法：范围 55~95mm")
    if not -8.0 <= args.tilt_trim <= 8.0:
        ap.error(f"--tilt-trim {args.tilt_trim} 非法：范围 -8~8°")
    if not sys.stdin.isatty():
        sys.exit("需要交互终端（ssh 加 -t；勿用 nohup/管道跑本脚本）")

    cfg = replace(DEFAULT_CONFIG, stand_height=args.stand_height,
                  cup_tilt_trim_deg=args.tilt_trim)
    if handover:
        cfg = replace(cfg, legs=tuple(
            replace(l, handover_mm=handover.get(l.name, 0.0)) for l in cfg.legs))
    if args.handover_rate is not None:
        cfg = replace(cfg, handover_rate_mms=args.handover_rate)
    if args.press_delta is not None:
        if not 8.0 <= args.press_delta <= 20.0:
            ap.error(f"--press-delta {args.press_delta} 非法：范围 8~20mm")
        cfg = replace(cfg, legs=tuple(
            replace(l, press_delta_mm=args.press_delta) for l in cfg.legs))

    log = RunLog(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs"),
        tag="mount")
    print(f"黑匣子日志: {log.path}")
    mode = "+".join(s for s, on in (("mock", args.mock), ("dry", args.dry),
                                    ("no-tank", args.no_tank)) if on) \
        or "实机全链路"
    log.note(f"模式={mode} port={args.port}")
    log.note(f"参数: wall_dist={args.wall_dist:g} wall_height="
             f"{args.wall_height if args.wall_height is not None else 'auto'}"
             f" rear_dist={args.rear_dist if args.rear_dist is not None else 'auto'}"
             f" pitch_step={args.pitch_step:g}"
             f" pitch_max={args.pitch_max:g} press_delta="
             f"{cfg.legs[0].press_delta_mm:g} stand={cfg.stand_height:g}"
             f" tilt_trim={cfg.cup_tilt_trim_deg:g} coxa_max={COXA_MAX_DEG:g}"
             f" belly={BELLY_MM:g} relay_first={int(args.relay_first)}")
    pwr = PowerWatch(log).start()
    if pwr.uv_ever_at_start:
        print("⚠ 本次开机以来 Pi 已出现过欠压（get_throttled 粘滞位）——供电有问题，"
              "先查 5V 降压再跑")
    step = startup_marker(log, pwr)
    _prev_hook = sys.excepthook

    def _crash_hook(tp, val, tb):
        pwr.stop()
        log.exc(val)
        log.close("uncaught")
        _prev_hook(tp, val, tb)
    sys.excepthook = _crash_hook

    step("打开舵机串口 + 继电器 GPIO17（保持断开）")
    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    if args.relay_first:
        servo_relay_close(drv, log, pwr)
    vent = None
    if args.mock or args.dry:
        io = MockVacuumIO(6)
        if args.dry:
            # 干跑也要真阀排气（walk_teleop 同款 GroundVent）：阀断电=通罐位，
            # 站起一压空气被挤过单向阀回不来，吸盘成被动真空把脚锁在地上，
            # 1 w 抬不起来（09-08 实测）。阀先于舵机出力，全程保持排气位
            vent = GroundVent(io_factory=lambda: Pi5VacuumIO(6, on_step=step))
            step("干跑：阀板初始化，六阀线圈按足串行通电（排气位，0.2s 间隔）")
            vent.set(True)
            step("干跑：六阀已到排气位（站起期间吸盘通大气；站定后断电，抬腿前按路通电）")
            log.note("dry=1 真阀按需排气：站起六阀通电→站定全断→抬腿那一路通电")
    else:
        step("阀板初始化：六阀线圈按足串行通电（排气位，0.2s 间隔）")
        io = Pi5VacuumIO(6, on_step=step)
        step("阀板/I2C 就绪")
    ctl_kw = dict(tankless=args.no_tank)
    if args.no_tank:
        ctl_kw["suck_timeout_s"] = 2.5
    ctl = AdhesionController(io, **ctl_kw)
    bot = Hexapod(drv, cfg)
    eng = MountEngine(cfg, ctl, front_hip_to_wall=args.wall_dist,
                      pitch_max_deg=args.pitch_max, attach_order=attach_order,
                      floor_clear_mm=args.floor_clear, support_only=support_only,
                      takeover_mm=takeover, support_tilt_deg=args.support_tilt,
                      slide_legs=slide_legs, slide_unload_mm=args.slide_unload)
    ho_txt = " ".join(f"{l.name}{l.handover_mm:g}" for l in cfg.legs
                      if l.handover_mm) or "关"
    tk_txt = " ".join(f"{n}{v:g}" for n, v in takeover.items() if v) or "只还欠账"
    log.note(f"handover={ho_txt} rate={cfg.handover_rate_mms:g}mm/s "
             f"takeover={tk_txt} step={args.takeover_step:g}")
    if support_only:
        log.note("support_only=" + ",".join(support_only))
    if slide_legs:
        log.note(f"slide_legs={','.join(slide_legs)} unload={args.slide_unload:g}")
    log.note("启动吸附序=" + "_".join(eng.attach_order))
    for n, v in wall_trim.items():
        deny = eng.set_wall_trim(v, [n])
        if deny:
            ap.error(f"--wall-trim {n}:{v:g} 不可行：{deny}")
    log.note("wall_trim=" + eng.trim_text())
    watch = ClimbWatch(log, eng, ctl, io, cfg)
    log.note(f"阈值: ATTACH={ATTACH_KPA} PUMP_ON={PUMP_ON_KPA}"
             f" PUMP_OFF={PUMP_OFF_KPA} suck_timeout={ctl.suck_timeout_s}s")

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    dt = 1.0 / cfg.update_hz
    peak_a = 0.0
    last_frozen = None
    clean_exit = False
    last_status = float("-inf")
    last_esc = float("-inf")
    last_o = float("-inf")
    released_hold = False
    sel = "L1"
    hover_was = None
    swing_was = None
    ho_was = (None, None)
    ho_note = None
    pose_was = False
    was_started = False
    at_pause = True
    aborted = False

    def pose_txt():
        return (f"俯仰{eng.pitch_deg:+.1f}° 高{eng.pose[1]:.0f} "
                f"前髋距墙{eng.front_hip_to_wall():.0f}")

    def say(msg, ev=None):
        print("\n" + msg)
        log.event(ev or msg)

    def dry_valve_tick():
        """干跑真阀按腿相位排气：摆动中（VENT→…→WAIT）与贴地滑动（SLIDE）线圈通电=排气，
        回 STANCE 或收在空中（AIR，吸盘悬空攒不出真空）断电。set_valve(False)=排气位=通电。"""
        vio = vent.io
        if vio is None:
            return
        for i, n in enumerate(LEG_NAMES):
            want_open = eng.phase_of[n] in VALVE_OPEN_PHASES
            if (not vio.valve[i]) != want_open:
                vio.set_valve(i, not want_open)
                log.event(f"干跑阀 {n} {'通电排气' if want_open else '断电'}"
                          f"（{eng.phase_of[n].value}）")

    def io_freeze(e):
        if not eng.frozen:
            eng.frozen = (f"IO 持续失败（{e}）——恢复后按 f；oo 取机 / "
                          "ESC×2 退出均可盲态运行（纯计时放气）")
            log.event(f"⚠ IO 降落伞：{e!r}")
            try:
                io.set_pump(False)
            except Exception:
                pass

    def do_move(name, surf, p_w, what):
        # 先算这一落点需要的关节姿态给操作者看（coxa 摆多少、倾角多少）
        deny = eng.request_move(name, surf, p_w)
        if deny:
            say(f"{what}拒绝：{deny}", f"{what}拒绝（{name}）：{deny}")
            return
        pb = eng.foot[name]
        sol = eng.geom[name].solve(
            tuple(eng.foot[name]), None)
        say(f"{name} → {what}：落点世界 ({p_w[0]:.0f},{p_w[1]:.0f},{p_w[2]:.0f})；"
            f"先通气抬起→弧线平移→面前 {cfg.lift_clearance:g}mm 悬停；"
            f"当前 coxa {sol['gamma']:+.0f}°。悬停后目视对正再按 i",
            f"{what}受理：{name} 落点 ({p_w[0]:.0f},{p_w[1]:.0f},{p_w[2]:.0f}) "
            f"{pose_txt()}")
        _ = pb

    def do_pose(dp=0.0, dx=0.0, dz=0.0, what=""):
        deny = eng.request_pose(dp, dx, dz)
        if deny:
            say(f"位姿拒绝（{what}）：{deny}", f"位姿拒绝（{what}）：{deny}")
        else:
            xb, zb, ph = eng._pose_to
            say(f"位姿 {what} 受理：→ 俯仰 {eng.pitch_deg + dp:+.1f}° 高 {zb:.0f} "
                f"前髋距墙 {eng.front_hip_to_wall(eng._pose_to):.0f}，约 "
                f"{eng._pose_T:.1f}s 铺完（空格停在半途）",
                f"位姿受理（{what}）：目标 俯仰{eng.pitch_deg + dp:+.1f}° 高{zb:.0f}"
                f" 前髋距墙{eng.front_hip_to_wall(eng._pose_to):.0f}")
            if eng._slide is not None:
                print(f"  随动 {'/'.join(sorted(eng._slide['legs']))}：开阀 {cfg.lift_vent_s:g}s → "
                      f"少压 {args.slide_unload:g}mm → 跟着机身滑 → 压回（铺完才算位姿铺完）")
            if eng.slide_note:
                print(f"  ⚠ {eng.slide_note}")
                log.event(f"⚠ 随动留痕：{eng.slide_note}")

    try:
        bot.move_feet(bot.crouch_feet(feet=eng.default_feet))
        if args.startup_gap > 0 and not args.mock:
            nxt = "固件使能" if args.relay_first else "舵机合闸"
            step(f"阀线圈已全通电→{nxt}前静置 {args.startup_gap:g}s（--startup-gap）")
            time.sleep(args.startup_gap)
        servo_power_on(drv, log, pwr, relay_closed=args.relay_first)
        print("缓慢站起（竖直升至爬墙站位，吸盘轴⊥地面）……")
        log.event("缓慢站起（竖直升至爬墙站位）")
        bot.glide_to(dict(eng.default_feet), 4.0)
        print("爬墙站位就位。")
        log.event(f"爬墙站位就位，进入就位暂停；{pose_txt()}")
        pwr.relax()
        if args.dry:
            # 站起时六盘被压缩已排完气，站定不动攒不出真空：现在把六线圈断掉省热。
            # 之后由 dry_valve_tick 按腿相位通电：进 VENT 起到回 STANCE 止
            vent.set(False)
            log.event("干跑：站定，六阀线圈断电；抬腿前按路自动通电排气")
            print("⚠ 干跑模式：吸附是仿真的、泵不动。站起时六阀已排气、现已断电；"
                  "哪条腿要抬就给那一路通电排气、落回支撑后断电（只热一路线圈）")
        if args.no_tank:
            print("⚠ 无罐模式：泵直抽歧管，没有储备真空——断电不保真空")
        print(f"起始位姿：{pose_txt()}（--wall-dist 量的是前腿 coxa 轴到墙面）")
        print("启动吸附序：" + "→".join(eng.attach_order)
              + ("（默认窗序）" if attach_order is None else "（--attach-order）"))
        if support_only:
            print("只承重不吸附：" + "/".join(support_only)
                  + f"——压到位即回支撑，不抽气、不算进互锁；抬腿前照样开阀放气 "
                    f"{cfg.lift_vent_s:g}s；倾角容差按 "
                    f"{args.support_tilt:g}°（只压不吸，不受吸盘密封的 15° 限制）。"
                    "⚠ 只能承压不能承拉，俯仰后别指望它们扛剥离力矩")
            if set(support_only) == set(LEG_NAMES):
                print("⚠ 六条腿都只承重：没有互锁，引擎也不算会不会翻——抬哪条腿全靠人看"
                      + ("" if args.dry or args.mock else
                         "；不是 --dry，六路阀会一直通电排气，线圈发热"))
        if slide_legs:
            print("随动腿：" + "/".join(slide_legs)
                  + f"——抬头/升降/平移机身时贴地跟着滑、保持相对机身的姿势；每段先开阀 "
                    f"{cfg.lift_vent_s:g}s、少压 {args.slide_unload:g}mm 再滑，铺完压回")
        if handover or takeover:
            dmax = max((l.handover_mm for l in cfg.legs), default=0.0)
            print(f"零力交接：δ={ho_txt}（{cfg.handover_rate_mms:g}mm/s，最长一段约 "
                  f"{dmax / cfg.handover_rate_mms:.1f}s）——抬腿前先把这条腿的力卸到零、"
                  "其余接触腿按份额接住，铺完才放气；落地吸住后反向做一次把载荷"
                  f"转给新腿（接管 {tk_txt}）")
            room = min(PRESS_DEPTH_MAX - l.press_delta_mm for l in cfg.legs)
            print(f"  ⚠ 地面腿的份额/接管量直接变成压深，余量只有 {room:.0f}mm"
                  "（PRESS_DEPTH_MAX−press_delta，要更大就调小 --press-delta）；"
                  "墙面腿是沿墙剪切、不吃压深，但吃剪切容量（15N）")
            print("  ⚠ 地面足的卸载方向就是把盘往上拔的方向：δ 给过头会把盘拔下来"
                  "（盘压一涨就是漏气挽救）——宁欠勿过，从实测弹跳×0.8 起标")
        else:
            print("零力交接：关（--handover 开；09-09 实测抬一只前足机身沉 26mm 不回弹）")
        for n in ("L1", "R1"):
            band = eng.wall_band(n)
            txt = (f"离地 {band[0]:.0f}~{band[1]:.0f}mm" if band else "无")
            print(f"  {n} 当前可落足带（墙面，倾角≤12°）：{txt}")
            log.note(f"{n} 可落足带={txt}")
        print("就位暂停：量前腿 coxa 轴到玻璃面的水平距离，与 --wall-dist 不符按 d "
              "输入实测值；确认无异常后按 p 开始全吸附启动序列（ESC×2 断电退出）")
        while True:
            k = read_key(0.1)
            if k == "d":
                print("\n输入前腿 coxa 轴到玻璃面的水平距离 mm（100~220，回车确认，ESC 取消）: ",
                      end="", flush=True)
                buf = ""
                while True:
                    c = read_key(0.5)
                    if c is None:
                        continue
                    if c == "\x1b":
                        buf = None
                        break
                    if c in ("\r", "\n"):
                        break
                    if c.isdigit() or (c == "." and "." not in buf):
                        buf += c
                        print(c, end="", flush=True)
                    elif c in ("\x7f", "\b") and buf:
                        buf = buf[:-1]
                        print("\b \b", end="", flush=True)
                try:
                    dval = float(buf) if buf else None
                except ValueError:
                    dval = None
                if dval is None or not 100.0 <= dval <= 220.0:
                    print("\n未修改（取消或超出 100~220）")
                    continue
                deny = eng.set_wall_dist(dval)
                if deny:
                    print(f"\n改距拒绝：{deny}")
                    continue
                args.wall_dist = dval
                print(f"\n前髋距墙改为 {dval:g}：{pose_txt()}")
                log.note(f"就位暂停按 d 改前髋距墙={dval:g}")
                for n in ("L1", "R1"):
                    band = eng.wall_band(n)
                    txt = (f"离地 {band[0]:.0f}~{band[1]:.0f}mm" if band else "无")
                    print(f"  {n} 可落足带（墙面，倾角≤12°）：{txt}")
                    log.note(f"{n} 可落足带={txt}")
                continue
            if k == "p":
                at_pause = False
                last_esc = float("-inf")
                print("开始全吸附启动序列……")
                log.event("按 p：开始全吸附启动序列")
                break
            if k == "o":
                print("\n尚未吸附（六阀已在排气位），没有要放开的吸盘；ESC×2 断电退出")
            if k == "\x1b":
                if time.monotonic() - last_esc < 2.0:
                    aborted = True
                    print("\n未开始吸附，断电退出。")
                    coils_off(io)
                    drv.close()
                    return
                last_esc = time.monotonic()
                print("再按一次 ESC 确认退出（尚未吸附；断电后请扶稳机身）")
            now = time.monotonic()
            if now - last_status > STATUS_S:
                last_status = now
                v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                    if args.mock else bot.check_power()
                peak_a = max(peak_a, c)
                watch.telemetry(v, c, peak_a, (0.0, 0.0, 0.0), note=" 就位暂停")

        t_wall = time.monotonic()
        while True:
            k = read_key(0)
            if k == "\x1b":
                if time.monotonic() - last_esc < 2.0:
                    clean_exit = True
                    break
                last_esc = time.monotonic()
                print("\n再按一次 ESC 确认退出（会放气——有足在墙上时先扶稳机身！）")
            elif k in ("UP", "DOWN", "LEFT", "RIGHT", "[", "]", "w", "g", "b",
                       "t", "h", "i", "z", ".", ",", "+", "=", "-") and released_hold:
                print("\n吸盘已放开（取机窗口），不可再动——取下后 ESC×2 退出")
            elif k in LEG_KEYS:
                sel = LEG_KEYS[k]
                print(f"\n已选 {sel}（{eng.phase_of[sel].value}）："
                      "w→墙 g→地面回位 b→正后方地面 t→站位前方地面 h 收起 i 落下"
                      " z 把载荷接管过来")
            elif k == "w":
                band = eng.wall_band(sel)
                if band is None:
                    say(f"{sel} 在当前位姿（{pose_txt()}）没有可落足带——"
                        "中/后腿要等俯仰≥50~75°，前腿检查 --wall-dist")
                else:
                    h = args.wall_height if args.wall_height is not None \
                        else (band[0] + band[1]) / 2.0
                    zp = eng.wall_perp_height(sel)
                    print(f"\n{sel} 可落足带 离地 {band[0]:.0f}~{band[1]:.0f}mm，"
                          f"取 {h:.0f}"
                          + ("" if args.wall_height is not None else "（带中点）"))
                    room = eng.wall_pitch_room(sel, h)
                    top = eng.wall_pitch_room(sel, band[1])
                    if zp is not None and room is not None:
                        print(f"  吸盘轴⊥墙在 {zp:.0f}mm。抬头余量几乎全押在落点高度："
                              f"落点每高 1mm 约多 0.45° 抬头，接管每 1mm 抵掉同样多"
                              f"（同一笔 15° 倾角预算）\n"
                              f"  此落点 + 接管 {eng.takeover.get(sel, 0.0):g}mm ⇒ "
                              f"{sel} 倾角允许抬到 {room:.0f}°"
                              + (f"；放到带上沿 {band[1]:.0f} 能到 {top:.0f}°"
                                 f"（代价：落点倾角变大、密封余量变小）"
                                 if top is not None and top > room else ""))
                    do_move(sel, eng.wall, eng.wall_target(sel, h), "上墙")
            elif k == "g":
                do_move(sel, FLOOR, eng.floor_home(sel), "地面回位")
            elif k == "b":
                rd = args.rear_dist if args.rear_dist is not None else eng.r0[sel]
                do_move(sel, FLOOR, eng.floor_back(sel, rd),
                        f"正后方 {rd:.0f}mm 地面")
            elif k == "t":
                p = eng.floor_forward(sel, args.fwd_dist, args.fwd_reach)
                reach_txt = ("" if args.fwd_reach is None
                             else f"、离髋 {args.fwd_reach:.0f}mm")
                if p is None:
                    say(f"{sel} 摆不到站位前方 {args.fwd_dist:g}mm{reach_txt}（前向分量超过"
                        "髋足距离）——减小 --fwd-dist 或加大 --fwd-reach",
                        f"站位前方拒绝（{sel}）：超出髋足距离")
                else:
                    do_move(sel, FLOOR, p, f"站位前方 {args.fwd_dist:.0f}mm{reach_txt} 地面")
            elif k == "h":
                deny = eng.request_tuck(sel)
                if deny:
                    say(f"收腿拒绝：{deny}", f"收腿拒绝（{sel}）：{deny}")
                else:
                    say(f"{sel} 收起悬空（抬 {cfg.lift_clearance:g}mm 后缩到髋旁；"
                        "不承载，互锁不算它；位姿改变时随身体）",
                        f"收腿受理：{sel} {pose_txt()}")
            elif k in (".", ","):
                # 只改"当前那条腿"：有悬停腿就是它（正对着它目测），否则改选中的腿
                tgt = eng.hover_leg or sel
                new = eng.wall_trim[tgt] + (2.0 if k == "." else -2.0)
                if not -20.0 <= new <= 40.0:
                    print(f"\n{tgt} 墙面修正 {new:+g} 超范围（-20~40）")
                else:
                    deny = eng.set_wall_trim(new, [tgt])
                    if deny:
                        say(f"{tgt} 墙面修正 {new:+g} 拒绝：{deny}（腿够不到了——机器人"
                            "离墙太远，或落点太高）", f"墙面修正拒绝 {tgt}:{new:+g}：{deny}")
                    else:
                        where = (f"悬停点随之{'贴近' if k == '.' else '远离'}墙 2mm"
                                 if eng.hover_leg == tgt and eng.surf[tgt] is eng.wall
                                 else "该腿不在墙面悬停，对它之后的墙面目标生效")
                        say(f"{tgt} 墙面修正 {new:+g}mm：{where}。目测到 15mm 再按 i；"
                            f"下次启动用 --wall-trim {eng.trim_text().replace(' ', ',')}",
                            f"墙面修正 {tgt}={new:+g}（全机 {eng.trim_text()}）{pose_txt()}")
            elif k in ("+", "=", "-"):
                hov = eng.hover_leg
                if hov is None:
                    print("\n没有悬停中的腿——落点高度只能在悬停时调（w 抬到悬停后）")
                else:
                    dz = 5.0 if k in ("+", "=") else -5.0
                    deny = eng.nudge_wall_target(hov, dz=dz)
                    if deny:
                        say(f"{hov} 落点上下挪 {dz:+g} 拒绝：{deny}",
                            f"落点挪动拒绝 {hov} dz={dz:+g}：{deny}")
                    else:
                        say(f"{hov} 落点{'升' if dz > 0 else '降'} {abs(dz):g}mm → 目标离地 "
                            f"{eng.target_height(hov):.0f}（{abs(dz) / 20.0:.1f}s 铺完，"
                            "铺完才让按 i）。与先放的那只目测齐平再按 i",
                            f"落点挪动 {hov} dz={dz:+g} → 目标离地 "
                            f"{eng.target_height(hov):.0f}")
            elif k == "i":
                hov = eng.hover_leg
                deny = eng.land()
                if deny:
                    say(f"落下不可用：{deny}")
                else:
                    say(f"{hov} 落下：沿面法向下探→压入 "
                        f"{cfg.leg(hov).press_delta_mm:g}mm→抽气确认",
                        f"落下：{hov} {pose_txt()}")
            elif k == "z":
                # 把载荷往选中的这条腿上转（连按累加）：L1 上墙吸住后，机身重量
                # 还压在被压弯的地面腿里，转过去=就地把它们攒的势能卸掉
                deny = eng.request_takeover(sel, args.takeover_step)
                if deny:
                    say(f"接管拒绝：{deny}", f"接管拒绝（{sel}）：{deny}")
                else:
                    surf_txt = ("墙面：吃的是剪切（实测容量 15N），盯盘压和电流"
                                if eng.surf[sel] is eng.wall
                                else f"地面：吃的是压深（余量到 {PRESS_DEPTH_MAX:g}mm 为止）")
                    say(f"{sel} 接管受理（{surf_txt}）",
                        f"接管受理：{sel} {args.takeover_step:g}mm 偏移={eng.ho_text()}")
            elif k == "UP":
                do_pose(dp=+args.pitch_step, what=f"抬头 {args.pitch_step:g}°")
            elif k == "DOWN":
                do_pose(dp=-args.pitch_step, what=f"低头 {args.pitch_step:g}°")
            elif k == "RIGHT":
                do_pose(dx=+LIN_STEP_MM, what=f"贴墙 {LIN_STEP_MM:g}mm")
            elif k == "LEFT":
                do_pose(dx=-LIN_STEP_MM, what=f"离墙 {LIN_STEP_MM:g}mm")
            elif k == "]":
                do_pose(dz=+LIN_STEP_MM, what=f"升 {LIN_STEP_MM:g}mm")
            elif k == "[":
                do_pose(dz=-LIN_STEP_MM, what=f"降 {LIN_STEP_MM:g}mm")
            elif k == " ":
                if eng.pose_pending:
                    eng.cancel_pose()
                    say(f"位姿铺设取消，停在 {pose_txt()}")
                elif eng.hover_leg:
                    print("\n悬停中的腿不可取消——按 i 落下，或 g/h 改去别处")
                else:
                    print("\n没有可取消的动作")
            elif k == "f" and released_hold:
                print("\n吸盘已放开（取机窗口），不可解冻——取下后 ESC×2 退出")
            elif k == "f" and eng.frozen:
                print(f"\n解除冻结: {eng.frozen}")
                eng.clear_freeze()
                last_frozen = None
            elif k == "o":
                # 取机窗口（与 body_lean/climb_walk 同款：逐足串行排气；改那边
                # 同步改这边）。有足在墙上时放开即坠，先扶稳
                if released_hold:
                    print("\n已是放开状态——取下后 ESC×2 退出")
                else:
                    if time.monotonic() - last_o < 2.0:
                        released_hold = True
                        eng.cancel_pose()
                        if not eng.frozen:
                            eng.frozen = ("取机窗口：吸盘已放开，姿态保持"
                                          "——取下后 ESC×2 退出")
                            last_frozen = eng.frozen
                        ctl.pump_inhibit = True
                        io.set_pump(False)
                        log.event("取机窗口：停泵 → 逐足串行排气（0.2s 间隔）"
                                  "→ 舵机撑住（泵禁开）")
                        print("\n放开吸盘：停泵 → 逐足串行排气（约 1.5s）……")
                        adh_dead = pwr_dead = False

                        def _pickup_tick():
                            nonlocal last_status, peak_a, adh_dead, pwr_dead
                            try:
                                ctl.update(dt)
                            except Exception as e:
                                if not adh_dead:
                                    adh_dead = True
                                    log.event("⚠ 取机期吸附状态机异常"
                                              f"（I2C 降级？），退化纯计时排气: {e}")
                            watch.poll()
                            t_now = time.monotonic()
                            if t_now - last_status > STATUS_S:
                                last_status = t_now
                                try:
                                    pv, pc = (drv.read_voltage_v(),
                                              drv.read_current_a()) \
                                        if args.mock else bot.check_power()
                                    peak_a = max(peak_a, pc)
                                    watch.telemetry(pv, pc, peak_a, (0, 0, 0),
                                                    note=" 取机放气")
                                except Exception as e:
                                    if not pwr_dead:
                                        pwr_dead = True
                                        log.event(f"⚠ 取机期电压读取失败: {e}")
                            if not args.mock:
                                time.sleep(dt)

                        for _ in range(int(0.3 / dt)):
                            _pickup_tick()
                        for i in range(6):
                            ctl.force_release(i)
                            io.set_valve(i, False)
                            for _ in range(int(0.2 / dt)):
                                _pickup_tick()
                        print("已放开：全阀排气、泵停，舵机撑住原地。"
                              "取下后 ESC×2 退出（阀线圈通电中，勿久放）")
                    else:
                        last_o = time.monotonic()
                        print("\n再按一次 o 确认放开全部吸盘（先冻结当前姿态，"
                              "舵机撑住）——有足在墙上=放开即坠，先扶稳机身"
                              "（安全绳兜底）再确认")

            if k is not None:
                log.event(f"键 {k!r} 选={sel} {pose_txt()}"
                          + ("（已放开）" if released_hold else ""))

            try:
                bot.move_feet(eng.update(dt))
            except WorkspaceError as e:
                if not eng.frozen:
                    eng.frozen = f"足端目标出工作空间（{e}）——ESC×2 退出"
            except OSError as e:
                io_freeze(e)
            watch.poll()
            if vent is not None:
                try:
                    dry_valve_tick()
                except OSError as e:
                    io_freeze(e)

            ho_now = eng.ho_leg
            ho_ph = eng.phase_of[ho_now] if ho_now else None
            if ho_now and (ho_now, ho_ph) != ho_was:
                sup = [n for n in eng.contact_legs() if n != ho_now]
                if ho_ph == MountPhase.HANDOVER:
                    d = cfg.leg(ho_now).handover_mm
                    say(f"{ho_now} 零力交接：竖直卸载 {d:g}mm、"
                        f"{'/'.join(sup)} 各接 {d / max(1, len(sup)):.1f}mm"
                        f"（约 {d / cfg.handover_rate_mms:.1f}s，仍吸附不放气）",
                        f"交接开始：{ho_now} δ={d:g} 支撑={'/'.join(sup)}")
                else:
                    d = eng.ho_span
                    say(f"{ho_now} 接管载荷 {d:.1f}mm：{'/'.join(sup)} 各松 "
                        f"{d / max(1, len(sup)):.1f}mm，身体指令不动"
                        f"（约 {d / cfg.handover_rate_mms:.1f}s）；累计偏移 "
                        f"{eng.ho_text()}",
                        f"接管开始：{ho_now} {d:.1f}mm 支撑={'/'.join(sup)}")
            ho_was = (ho_now, ho_ph)
            if eng.handover_note != ho_note:
                ho_note = eng.handover_note
                if ho_note:
                    print(f"\n⚠ {ho_note}")
                    log.event(f"⚠ 交接留痕：{ho_note}")
            hover_now = eng.hover_leg
            if hover_now and hover_now != hover_was:
                fw = eng._foot_world(hover_now)
                on_wall = eng.surf[hover_now] is eng.wall
                # 离面距离**按面取**：墙面 lift_clearance（15），地面 --floor-clear
                # （默认 45，腿一抬还会自重下垂十几到二十几毫米）。09-11 实机这行
                # 对地面悬停印的是 15 而实际 45——操作者拿它当目测基准，不能印错
                clr = cfg.lift_clearance if on_wall else eng.floor_clear
                print(f"\n{hover_now} 已悬停：世界 ({fw[0]:.0f},{fw[1]:.0f},"
                      f"{fw[2]:.0f})，离{'墙' if on_wall else '地'} {clr:g}mm"
                      + (f"（{hover_now} 墙面修正 {eng.wall_trim[hover_now]:+g}）"
                         if on_wall else "")
                      + "——目视吸盘对正/间距，i 落下压入；"
                      + (f"间距不是 {clr:g} 就 . ,（每次 2mm）补到 {clr:g}；"
                         "与另一只前盘不齐平就 +/-（每次 5mm）挪落点高度；"
                         if on_wall else "")
                      + "不对就 g/h 挪走")
                log.event(f"悬停：{hover_now} 世界 ({fw[0]:.0f},{fw[1]:.0f},{fw[2]:.0f})")
            hover_was = hover_now
            swing_now = eng.swing_leg
            if swing_was and not swing_now:
                st = eng.surf[swing_was]
                where = ("空中" if st is None else
                         {"floor": "地面", "wall": "墙面"}.get(st.name, st.name))
                mark = ("只承重（不吸附）" if swing_was in support_only else
                        "已吸附" if ctl.is_attached(LEG_NAMES.index(swing_was)) else "")
                say(f"{swing_was} 收口：{where} {mark}"
                    f"；接触腿 {'/'.join(eng.contact_legs())}",
                    f"收口：{swing_was}→{where} 接触腿={'/'.join(eng.contact_legs())}")
            swing_was = swing_now
            pose_now = eng.pose_pending
            if pose_was and not pose_now:
                say(f"位姿铺完：{pose_txt()}", f"位姿铺完：{pose_txt()}")
            pose_was = pose_now

            if eng.started and not was_started:
                was_started = True
                print(f"\n✓ 六足吸附完成（{pose_txt()}）：1~6 选腿  w 上墙  g 回地  "
                      "b 正后方  t 站位前方  h 收起  i 落下  z 接管载荷  ./, 离墙"
                      "  +/- 落点高低  ↑/↓ 俯仰  ←/→ 离/贴墙  [/] 降/升"
                      "  空格取消位姿  f 解冻  o×2 取机  ESC×2 退出")
            if eng.frozen != last_frozen:
                last_frozen = eng.frozen
                if eng.frozen:
                    print(f"\a\n⚠⚠⚠ 全机冻结: {eng.frozen} —— 处理后按 f 继续\n")

            now = time.monotonic()
            if now - last_status > STATUS_S:
                last_status = now
                try:
                    v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                        if args.mock else bot.check_power()
                except OSError as e:
                    io_freeze(e)
                else:
                    peak_a = max(peak_a, c)
                    tag = (f" {pose_txt()} 选{sel}"
                           + (f" 空中{'/'.join(n for n in LEG_NAMES if eng.surf[n] is None and eng.phase_of[n] == MountPhase.AIR)}"
                              if any(eng.phase_of[n] == MountPhase.AIR for n in LEG_NAMES) else "")
                           + (f" 交接{eng.ho_text()}" if eng.ho_text() != "0" else "")
                           + (" 已放开" if released_hold else ""))
                    print("\r" + status_line(eng, ctl, v, c, peak_a,
                                             (0.0, 0.0, 0.0), tag) + "  ",
                          end="", flush=True)
                    watch.telemetry(v, c, peak_a, (0.0, 0.0, 0.0), note=tag)
            lag = time.monotonic() - t_wall
            if lag > 5 * dt:
                log.event(f"⚠ 主环卡顿 {lag * 1000:.0f}ms（目标 {dt * 1000:.0f}ms）")
            time.sleep(max(0.0, dt - lag))
            t_wall = time.monotonic()
    except KeyboardInterrupt:
        log.event("Ctrl-C 中断")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if aborted:
            log.event("退出：就位暂停处确认退出（未吸附，已断电）")
        elif at_pause:
            coils_off(io)
            log.event("中断：就位暂停处（未吸附，阀线圈已断电，舵机保持使能）")
            print("\n在就位暂停处中断：未吸附，阀线圈已断电。"
                  "舵机断电请关电源或跑 climb_walk --release。")
        elif clean_exit:
            # 退出序列与 body_lean/climb_walk 同款（逐足串行放气、盲态计时兜底、
            # coils_off 必达；改那边同步改这边）
            log.event(f"退出序列：停泵 → 逐足串行放气 → 舵机断电（不回站姿）；{pose_txt()}")
            print("\n退出：停泵 -> 逐足串行放气 -> 舵机断电"
                  "（停在当前姿态，约 5s；Ctrl-C 已屏蔽，等它跑完）")
            prev_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                ctl.pump_inhibit = True
                io.set_pump(False)
                if not args.mock:
                    time.sleep(0.3)
                t_tlm = float("-inf")
                adh_dead = False

                def _exit_tick():
                    nonlocal t_tlm, peak_a, adh_dead
                    try:
                        ctl.update(dt)
                        watch.poll()
                    except Exception as e:
                        if not adh_dead:
                            adh_dead = True
                            log.event("⚠ 退出期吸附状态机异常（I2C 降级？），"
                                      f"退化纯计时放气: {e}")
                    if time.monotonic() - t_tlm > STATUS_S:
                        t_tlm = time.monotonic()
                        try:
                            v, c = drv.read_voltage_v(), drv.read_current_a()
                            peak_a = max(peak_a, c)
                            watch.telemetry(v, c, peak_a, (0, 0, 0),
                                            note=" 退出放气")
                        except Exception as e:
                            log.event(f"⚠ 退出期电压读取失败，停采: {e}")
                            t_tlm = float("inf")

                for i in range(6):
                    ctl.force_release(i)
                    for _ in range(int(1.5 / dt)):
                        _exit_tick()
                        if not args.mock:
                            time.sleep(dt)
                        if ctl.state[i] == FootState.RELEASED:
                            break
                    else:
                        ctl.abandon_release(i)
                        log.event(f"⚠ {LEG_NAMES[i]} 排气 1.5s 未确认"
                                  "（堵/传感器漂移？），线圈照断继续")
                    io.set_valve(i, True)
                    for _ in range(int(0.2 / dt)):
                        _exit_tick()
                        if not args.mock:
                            time.sleep(dt)
            finally:
                coils_off(io)
                log.event("退出：阀线圈已断电（coils_off）")
                drv.close()
                signal.signal(signal.SIGINT, prev_int)
            print("完成（阀线圈已断电）。")
            log.event("退出序列完成（舵机断电）")
        else:
            log.event(f"中断：不放气退出，冻结={eng.frozen or '无'}")
            print(f"\n中断：不放气退出。冻结: {eng.frozen or '无'}；"
                  "善后请跑 climb_walk --release。")
        if vent is not None:
            # 干跑的真阀：六线圈断电再释放，绝不拉高着退（阀一直通电发热，08-17）
            try:
                vent.close()
                log.event("干跑：六阀线圈已断电（GroundVent.close）")
            except Exception as e:
                log.event(f"⚠ 干跑阀断电失败: {e}")
        pwr.stop()
        log.close("正常退出" if (clean_exit or aborted) else "中断退出")


if __name__ == "__main__":
    main()
