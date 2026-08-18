"""黑匣子日志（hexapod.runlog）测试：文件落盘、吸附状态机钩子、跳变轮询、
遥测行与电源告警沿。全部跑在 MockVacuumIO + 无硬件路径上。"""
from hexapod.adhesion import (AdhesionController, MockVacuumIO, ATTACH_KPA)
from hexapod.climb import ClimbEngine
from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.runlog import RunLog, ClimbWatch

DT = 0.02


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def make():
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    return io, ctl, ClimbEngine(CFG, ctl)


def test_runlog_header_events_rows_and_open_after_end(tmp_path):
    log = RunLog(str(tmp_path), tag="t")
    log.note("参数 abc=1")
    log.event("事件甲")
    log.row("t=1.0 x=2")
    log.close("测试")
    txt = read(log.path)
    assert "argv:" in txt and "# 参数 abc=1" in txt
    assert "EVT 事件甲" in txt and "TLM t=1.0 x=2" in txt
    assert "EVT END 测试" in txt
    # close 不封死文件：清理路径（finally/excepthook）先后不定，后到的追记
    # 仍要写得进去；END 只写一次
    log.event("尾巴")
    log.close("再次")
    txt = read(log.path)
    assert "尾巴" in txt and txt.count("EVT END") == 1


def test_runlog_exc_only_once(tmp_path):
    log = RunLog(str(tmp_path), tag="t")
    try:
        raise ValueError("炸")
    except ValueError as e:
        log.exc(e)
        log.exc(e)
    txt = read(log.path)
    assert txt.count("ValueError: 炸") == 1 and "异常退出" in txt


def test_adhesion_hook_fires_with_last_kpa():
    io = MockVacuumIO(1)
    io.tank_kpa = -75.0          # 罐压现成，吸附流程确定性走完
    ctl = AdhesionController(io, n_feet=1)
    seen = []
    ctl.on_event = lambda i, old, new, el: seen.append((i, old.value,
                                                        new.value, el))
    ctl.request_attach(0)
    for _ in range(200):
        ctl.update(DT)
        if ctl.is_attached(0):
            break
    trans = [(o, s) for _, o, s, _ in seen]
    assert ("released", "pressing") in trans
    assert ("pressing", "sucking") in trans
    assert ("sucking", "attached") in trans
    assert all(el >= 0.0 for *_, el in seen)
    # 判据现场镜像：attach 判定必然刚读过足压
    assert ctl.last_kpa[0] is not None and ctl.last_kpa[0] <= ATTACH_KPA


def test_climbwatch_startup_edges_and_telemetry(tmp_path):
    io, ctl, eng = make()
    log = RunLog(str(tmp_path), tag="t")
    watch = ClimbWatch(log, eng, ctl, io, CFG)
    for _ in range(int(30 / DT)):
        eng.update(DT)
        watch.poll()
        if eng.started:
            break
    assert eng.started
    watch.telemetry(7.40, 2.0, 3.0, (15.0, 0.0, 0.0), note=" 测试")
    txt = read(log.path)
    assert "启动序列完成" in txt
    assert "EVT 泵 开" in txt                       # 冷罐建压必开泵
    assert "吸附 L1 sucking→attached" in txt       # 钩子带判据现场
    assert "kPa" in txt
    assert "相位 L1 wait→stance" in txt
    assert "阀 L1 →抽气位" in txt
    assert "TLM" in txt and "7.40V" in txt and "测试" in txt
    # 电压告警沿：跌破 warn 记一次，回差内不重复
    watch.telemetry(6.30, 2.0, 3.0, (0.0, 0.0, 0.0))
    watch.telemetry(6.31, 2.0, 3.0, (0.0, 0.0, 0.0))
    txt = read(log.path)
    assert txt.count("电压告警") == 1


def test_climbwatch_freeze_edge(tmp_path):
    io, ctl, eng = make()
    log = RunLog(str(tmp_path), tag="t")
    watch = ClimbWatch(log, eng, ctl, io, CFG)
    eng.frozen = "测试冻结"
    watch.poll()
    eng.frozen = None
    watch.poll()
    txt = read(log.path)
    assert "冻结 ⚠ 测试冻结" in txt and "EVT 解冻" in txt
