"""地爬墙脚本的状态行与收尾——2026-09-13 从 scripts/climb_walk.py 复制，此后各改各的。"""
from hexapod.config import LEG_NAMES
from hexapod.runlog import PHASE_CH, ADH_CH


def status_line(eng, ctl, v, c, peak, cmd, tag=""):
    legs = " ".join(
        f"{n}{PHASE_CH[ph]}{'!' if leak else ADH_CH[adh]}"
        for n, (ph, adh, leak) in eng.status().items())
    # 盘差 = 最差（最接近大气）的已吸附盘压 kPa：真正承载的是它。读数一律取
    # 状态机镜像（ctl.last_*，与黑匣子遥测同源）：显示路径零传感器 IO——
    # 不抢 I2C 突发配额，也绝不因转换超时抛 IOError 炸主环
    att = [(ctl.last_kpa[i], LEG_NAMES[i])
           for i in range(6) if ctl.is_attached(i)
           and ctl.last_kpa[i] is not None]
    cup_txt = "盘差 {1}{0:6.1f}".format(*max(att)) if att else "盘差 --"
    if ctl.tankless:
        tank_txt = "罐 无罐"          # 无罐模式不读罐压传感器（读了也是悬空假数）
    else:
        tf = " 罐压失效!" if ctl.tank_fault else ""
        tank_txt = (f"罐 {ctl.last_tank_kpa:6.1f}kPa{tf}"
                    if ctl.last_tank_kpa is not None else f"罐 --{tf}")
    # 泵开/停取 io.pump 内存镜像（set_pump 写入，与 TLM 泵字段同源）：零传感器 IO
    pump_txt = "泵开" if getattr(ctl.io, "pump", False) else "泵停"
    head = ("启动" if not eng.started else f"t={eng.t:5.1f}") + tag
    vx, vy, wz = cmd
    sp = "停" if not (vx or vy or wz) else f"{vx:+.0f}/{vy:+.0f}/{wz:+.2f}"
    return (f"[{head}] {legs}  速 {sp}  {cup_txt}  {tank_txt}  {pump_txt}  "
            f"{v:.2f}V {c:5.2f}A 峰 {peak:5.2f}A")


def coils_off(io):
    """退出前必做：六路阀线圈全部断电（GPIO 拉低）+ 泵停。
    本机阀是"通电=排气、断电=通罐"，排气是**维持态**——退出序列把 GPIO
    拉高放气后就退进程，引脚会停在高电平，六个线圈一直通电发热（实机
    2026-08-17 复现）。收尾必须全部拉低再走。
    注意 set_valve(True) 恰是线圈断电电平（True=通罐位=低电平）；罐里若有
    残余真空，地面吸盘可能被轻微重新吸住，关 12V 或重跑 --release 可放开。"""
    for i in range(6):
        io.set_valve(i, True)
    io.set_pump(False)
