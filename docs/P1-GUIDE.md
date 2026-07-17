# P1 吸附系统台架验证 · 详细操作指南

目标（第 2~5 周）：**在工作台上把"泵 → 罐 → 阀 → 吸盘 + 传感器 → Pi"整条真空链路调通，
拿到爬墙可行性的第一批实测数据。**
本阶段结束时你应该能拍出一段视频：竖直玻璃上单吸盘足挂着 3kg 配重纹丝不动，
SSH 进 Pi 敲一条命令，完成一个"吸-确认-放"循环（<2s）。

顺序建议：第 1 步不依赖气路件，舵机装好就能做；第 3~7 步严格按序（每步是下一步的前提）；
第 8~9 步是产出数据的正戏。

---

## 第 1 步 · 单腿 ±45° 脉宽标定（等气路件期间做完）

P2 的 `single_leg_wall.py` 要做单腿 IK，L1 腿（通道 15/16/17）的标定是前提。
流程见 `software/README.md`"标定流程"，这里补充台架细节：

1. **舵机供电电压**再确认一次：不得超过舵机标签上限（6.8V 版必须经降压给 6.0~6.5V，
   不能 2S 直连）。
2. 逐关节找 -45°/+45° 的实际脉宽，两个方法选一：
   - **方法 A（不拆结构，推荐）**：手机水平仪贴在腿段上。先回中记下基准角，
     然后微调脉宽直到腿段相对基准转过 ±45°。俯仰关节（femur/tibia）用水平仪，
     水平摆动的 coxa 用垫在下面的纸打印量角器。
   - **方法 B（更准，费事）**：拆下腿段打印件，舵盘装 P0 打印的 `calibration-arm`，
     配 `calibration-ruler` 按官方视频用法找 ±45° 刻度。
   微调脉宽用 python 交互：
   ```bash
   cd ~/spider/software && source .venv/bin/activate && python
   >>> from hexapod.driver import Servo2040Driver
   >>> d = Servo2040Driver(); d.set_all_pulses_us([1500]*18); d.enable(True)
   >>> d.set_pulses_us(16, [1520])   # femur=16，±10µs 一步慢慢逼近
   ```
3. 实测值填入 `config.py` L1 的 `us_m45/us_p45`；方向反了改 `sign=-1`，
   零位残差微调 `attach_deg`。改完 `pytest tests/` 必须仍然全绿。
4. 复核：命令 femur 到 0°、+45°，水平仪实测误差 <2° 即合格。

## 第 2 步 · 气路件到货核验（到货当天）

| 器件 | 核对项 |
|---|---|
| 二位三通电磁阀 | 12V、"常闭"标称——真实行为第 3 步实测，别只信页面 |
| 30mm 波纹吸盘 | 金具是 **M5 通孔**（气从螺柱中心走）+ 90° 弯头宝塔，缺一样 `suction_foot` 装不了 |
| XGZP6847A | **模拟输出版**（3 线：5V/GND/Vout），不是 I2C 版；量程 0~-100kPa |
| 555 双头泵 | 12V、标称极限 ≤-75kPa；两个抽气头，可三通并联提速 |
| 单向阀 | 壳体有流向箭头 |
| 8 路 MOSFET 板 | 触发电平兼容 3.3V（光耦版看说明，"高电平触发 3-24V"即可） |
| XL6009 | 可调版（带电位器） |

## 第 3 步 · 电磁阀特性实测（半小时，决定接法、软件极性和一个设计假设）

设计文档假设"常闭阀断电 = 吸盘保持真空"（`CLIMBING-DESIGN.md` §2 的被动安全设计）。
但市售 0520B 类"常闭"三通阀不少是**断电时出气口通大气**——真是这样的话断电悬停
不成立。这一步就是把买到的阀摸清楚：

1. 万用表量线圈电阻（几十 Ω 正常），12V 上电听到"咔哒"说明线圈工作。
2. **吹气法**测通断：三个气口分别标 P（接罐/真空侧）、A（接吸盘侧）、R（排气口）。
   断电状态和通电状态下，分别往每个口吹气，记录哪两个口连通：

   | 状态 | 实测连通关系 |
   |---|---|
   | 断电 | ___ ↔ ___ |
   | 通电 | ___ ↔ ___ |

3. 对照结论：
   - **理想**（断电 A↔P 或 A 全封闭）：断电保持真空成立，按此接管。
   - **常见**（断电 A↔R 通大气）：P1 台架不受影响照常做（数据测试时阀保持通电即可），
     但要把实测结果记进 `CLIMBING-DESIGN.md` §4——P4 前必须解决
     （换"断电保持"接法的阀型，或修改断电悬停的安全论证）。
4. 无论哪种，写下"**吸盘接通真空**对应线圈通电还是断电"——第 7 步 `VALVE_ON_LEVEL`
   常量由它决定。

## 第 4 步 · 12V 供电与 MOSFET 驱动链路（半天）

1. **XL6009 调压**：输入接 2S 电池（接前万用表确认极性），**空载**电位器调到输出
   12.0V，再接负载。P1 负载（1 阀 + 1 泵）约 1A，3A 模块足够。
2. **MOSFET 板接线**：电源侧 VCC=12V、GND 与 Pi 的 GND **共地**；控制侧
   IN1 ← Pi GPIO5（阀）、IN7 ← GPIO20（泵），与 `adhesion.py` 里
   `VALVE_PINS[0]=5`、`PUMP_PIN=20` 对应（改接了就同步改常量）。
3. 先不接阀和泵，GPIO 点动看板上指示灯/输出端电压：
   ```bash
   sudo apt install -y python3-lgpio gpiod
   pip install lgpio smbus2        # 即 pyproject 的 [pi] 附加依赖
   python - <<'EOF'
   import lgpio, time
   h = lgpio.gpiochip_open(0)      # 报错就试 4；gpioinfo | grep pinctrl-rp1 确认编号
   for pin in (5, 20):
       lgpio.gpio_claim_output(h, pin, 0)
       lgpio.gpio_write(h, pin, 1); time.sleep(1); lgpio.gpio_write(h, pin, 0)
   lgpio.gpiochip_close(h)
   EOF
   ```
4. 通过后接上阀和泵再点动一遍：阀咔哒、泵转。

## 第 5 步 · 压力传感链路（半天）

XGZP6847A 输出 0.5~4.5V，超过 Pi 的 3.3V 体系，**必须分压**后进 ADS1115：

| 连接 | 说明 |
|---|---|
| XGZP 5V / GND | 接 Pi 引脚 2(5V) / 6(GND) |
| XGZP Vout → 10k → A0，A0 → 10k → GND | 1:1 分压，4.5V→2.25V |
| ADS1115 VDD/GND | 接 Pi 3.3V / GND（**VDD 接 3.3V，不是 5V**） |
| ADS1115 SDA/SCL | Pi 引脚 3 / 5（I2C1） |
| ADS1115 ADDR → GND | 地址 0x48 |

1. `i2cdetect -y 1` 应看到 `48`。
2. 读电压：
   ```python
   from smbus2 import SMBus
   import time
   def read_a0_v(addr=0x48):
       with SMBus(1) as bus:
           bus.write_i2c_block_data(addr, 0x01, [0xC3, 0x83])  # A0 单端 ±4.096V 单次
           time.sleep(0.01)
           hi, lo = bus.read_i2c_block_data(addr, 0x00, 2)
           raw = (hi << 8) | lo
           return (raw - 65536 if raw > 32767 else raw) * 4.096 / 32768
   print(read_a0_v() * 2)   # ×2 = 分压前的传感器电压
   ```
3. **两点定标**：
   - 大气下记录传感器电压 `V_ATM`（0~-100kPa 版应接近 4.5V）；
   - 用手（或下一步装好的泵）抽到泵极限，电压应单调走向另一端，
     按 (V_ATM, 0kPa) 和 (极限电压, 泵标称 -75kPa 附近) 两点确认斜率符号，
     满量程斜率标称 100kPa/4V = 25：`kPa = 25 × (V − V_ATM)`（抽气电压下降的批次），
     方向反了斜率取负。真空度必须换算出**负值**。
   - 把 `V_ATM` 和斜率记下来，第 7 步回填。

## 第 6 步 · 组装真空回路 + 手动验证（不经软件，半天）

P1 单足回路（`CLIMBING-DESIGN.md` §4 气路图的单足子集）：

```
泵(抽气口) ← 单向阀 ← 储气罐(PET瓶) ← 三通阀[P口] ；三通阀[A口] → 三通 ┬→ 吸盘
                                        三通阀[R口] = 大气              └→ XGZP6847A
```

1. **储气罐**：碳酸饮料 PET 瓶（耐负压，普通矿泉水瓶会吸瘪）。瓶盖钻两孔插
   4mm 管，热熔胶+环氧双重密封。
2. **单向阀方向**：箭头（气流方向）指向泵——即允许"罐→泵"抽气、阻止大气倒灌。
   装好后停泵，罐压不回升即装对。
3. **传感器位置**：P1 只有 1 个 XGZP，用三通接在**阀-吸盘之间**（吸盘支路的压力
   才是验收对象）。想同时监控罐压就再买一个（¥20，BOM 表 2 本来就写 3~6 个）。
4. 接头全部：插到底 + 扎带锁紧；宝塔接头处可加一圈生料带。
5. **手动验证**（跳过软件，隔离问题）：直接给泵 12V，吸盘按在擦净的玻璃上，
   看第 5 步的电压读数——应能到 -60kPa 以下。到不了就肥皂水抹接头找漏，
   或逐段夹管二分法定位。

## 第 7 步 · 实现 Pi5VacuumIO 并联调（1~2 天）

把 `software/hexapod/adhesion.py` 里的 `Pi5VacuumIO` 占位类替换为实现
（常量按第 3/5 步实测回填）：

```python
class Pi5VacuumIO:
    """树莓派 5 实机 IO。P1 台架:1 阀 1 泵 1 传感器;P4 扩到 6 阀 2 泵。"""
    VALVE_PINS = [5]
    PUMP_PIN = 20
    VALVE_ON_LEVEL = 1     # set_valve(True)=吸盘接通真空 的 GPIO 电平,按第 3 步实测
    ADS_ADDR = 0x48
    V_DIV = 2.0            # 1:1 电阻分压
    V_ATM = 4.50           # 第 5 步实测大气点电压
    KPA_PER_V = 25.0       # 第 5 步实测斜率(含符号)
    GPIOCHIP = 0           # 打不开就改 4

    def __init__(self, n_feet=1):
        import lgpio
        from smbus2 import SMBus
        self._lg, self.n = lgpio, n_feet
        self._h = lgpio.gpiochip_open(self.GPIOCHIP)
        for p in self.VALVE_PINS[:n_feet]:
            lgpio.gpio_claim_output(self._h, p, 1 - self.VALVE_ON_LEVEL)
        lgpio.gpio_claim_output(self._h, self.PUMP_PIN, 0)
        self._bus = SMBus(1)

    def set_valve(self, i, on):
        self._lg.gpio_write(self._h, self.VALVE_PINS[i],
                            self.VALVE_ON_LEVEL if on else 1 - self.VALVE_ON_LEVEL)

    def set_pump(self, on):
        self._lg.gpio_write(self._h, self.PUMP_PIN, 1 if on else 0)

    def _read_v(self, ch=0):
        import time
        self._bus.write_i2c_block_data(self.ADS_ADDR, 0x01, [0xC3 + (ch << 4), 0x83])
        time.sleep(0.01)
        hi, lo = self._bus.read_i2c_block_data(self.ADS_ADDR, 0x00, 2)
        raw = (hi << 8) | lo
        return (raw - 65536 if raw > 32767 else raw) * 4.096 / 32768

    def read_foot_kpa(self, i):
        return self.KPA_PER_V * (self._read_v(0) * self.V_DIV - self.V_ATM)

    def read_tank_kpa(self):
        # P1 单传感器在吸盘支路,罐压暂用同一读数近似——泵的滞环控制因此不准,
        # 台架测试期间泵手动控制或常开即可;P4 罐上加独立传感器后恢复滞环。
        return self.read_foot_kpa(0)

    def close(self):
        self._bus.close()
        self._lg.gpiochip_close(self._h)
```

改完先跑 `pytest tests/`（Mock 路径必须不受影响），再上台架跑完整状态机循环：

```python
import time
from hexapod.adhesion import AdhesionController, FootState, Pi5VacuumIO
io = Pi5VacuumIO(n_feet=1); ctl = AdhesionController(io, n_feet=1)
io.set_pump(True); time.sleep(3)            # 先把罐抽起来（泵手动控制）
t0 = time.time(); ctl.request_attach(0)     # 吸盘此时应已压在玻璃上
while not ctl.is_attached(0):
    ctl.update(0.02); time.sleep(0.02)
    if ctl.state[0] is FootState.FAULT:
        raise SystemExit("SUCKING 超时——没吸住,查密封")
ta = time.time() - t0
print(f"吸附确认 {ta:.2f}s  压力 {io.read_foot_kpa(0):.1f} kPa")
time.sleep(3)
t1 = time.time(); ctl.request_release(0)
while ctl.state[0] is not FootState.RELEASED:
    ctl.update(0.02); time.sleep(0.02)
tr = time.time() - t1
print(f"释放 {tr:.2f}s  循环合计 {ta+tr:.2f}s (验收目标 <2s)")
io.set_pump(False); io.close()
```

## 第 8 步 · 三条数据曲线（竖直玻璃，1 天）

玻璃/瓷砖先酒精擦净。压力记录脚本（20Hz 存 CSV，测完 scp 回开发机看）：

```python
import time, csv, sys
from hexapod.adhesion import Pi5VacuumIO
io = Pi5VacuumIO(n_feet=1)
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "kpa"]); t0 = time.time()
    while time.time() - t0 < 90:
        w.writerow([round(time.time()-t0, 3), round(io.read_foot_kpa(0), 2)])
        time.sleep(0.05)
```

| 曲线 | 操作 | 目标 | 实测 |
|---|---|---|---|
| 1 抽气时间 | 吸盘贴壁 → 开阀，计时到 -40kPa | <1s | ___ |
| 2 泄漏率 | 到 -40kPa 后停泵、阀置"保持"态，看回升 | 60s 内不高于 -20kPa | ___ |
| 3 释放时间 | 阀切放气 → 吸盘可无阻力拿开 | <0.5s | ___ |

不达标排查：曲线 1 慢 → 双头泵并联/管路缩短/罐先抽到位；曲线 2 差 → 肥皂水找漏、
吸盘唇口和玻璃重新擦、接头补胶；曲线 3 慢 → 排气口是否被堵、管径。

## 第 9 步 · 挂重、斜角、打印件复测（1~2 天，P2 的决策数据）

**安全**：配重正下方不站人不放脚，地面垫软物；配重用水瓶/砝码逐级加；建议护目镜
（吸盘崩脱会弹）。

1. **法向拉脱**：玻璃水平架起，吸盘吸在下表面，向下逐级挂重到脱落。
   理论参照 ~35N（3.5kg）@-50kPa。记录：___ kg @ ___ kPa
2. **剪切拉脱**：玻璃竖直，吸盘吸上去侧向挂重到脱落。理论约法向 5 折。
   记录：___ kg @ ___ kPa
3. **斜角容差**（决定爬墙步幅，`CLIMBING-DESIGN.md` §6 的关键输入）：
   垫 5°/10°/15°/20° 楔块（打印或木楔），每个角度做 10 次吸附：

   | 倾角 | 密封成功次数/10 | 拉脱力 |
   |---|---|---|
   | 5° / 10° / 15° / 20° | | |

   结论回填 §6：容差 ≥15° → 步幅可用 40mm；只有 10° → 收紧到 ~25mm。
4. **suction_foot 打印件复测**：吸盘金具穿过打印件底盘锁 M5 螺母（侧窗拧），
   弯头宝塔从侧窗出管，整足装到 tibia 方轴上（顶丝锁紧），重复曲线 2 和挂重。
   打印件层间漏气 → 螺母腔和底盘内面刷薄环氧/侧窗封胶；强度不够 → 加壁重打。
5. **称重**：整足（打印件+吸盘+金具+顶丝）记入 `docs/weight-log.md`。

## P1 验收核对表

- [ ] L1 腿 ±45° 标定填入 `config.py`，`pytest` 全绿，实测复核 <2°
- [x] 电磁阀断电/通电行为实测记录在案；若"断电≠保持真空"已记入 CLIMBING-DESIGN 待办
- [x] `Pi5VacuumIO` 实装（GPIO 极性、压力换算经两点定标）
- [x] 曲线 1/2/3 达标：<1s 到 -40kPa；60s 不回升过 -20kPa；释放 <0.5s
- [x] 法向/剪切拉脱力、斜角容差数据齐全，步幅结论回填 §6
- [x] **竖直玻璃挂 1.5kg (30mm吸盘极限) 保持 10 分钟**（用 suction_foot 打印件整足做）
- [x] **"吸-确认-放"循环 <2s**（状态机跑通，非手动）
- [x] 整足重量记入 weight-log
- [ ] P2 备料下单：L 形支架材料、导轨/抽屉滑轨+滑车、1.5kg 配重、竖直玻璃板

全部勾选 → 进入 P2（单腿爬墙决策门）。P1/P2 期间打印机空闲，可开始打整机结构件。

## 常见问题

| 症状 | 处理 |
|---|---|
| `i2cdetect` 看不到 0x48 | SDA/SCL 接反；ADDR 悬空（默认 0x48 但要接 GND 确认）；I2C 没开（P0 已开过，`raspi-config` 复查） |
| 压力读数恒定不变 | 分压电阻没接/接错通道 A0；传感器 5V 供电没接 |
| 状态机永远到不了 ATTACHED | 换算符号错了——真空度必须是负值，检查 `KPA_PER_V` 符号 |
| 阀不动作 | 12V 实测（XL6009 带载跌压）；MOSFET 触发电平与共地；线圈电阻是否开路 |
| 泵转但抽不动 | 双头泵接错口（抽气口 vs 排气口）；单向阀装反；管路某接头大漏 |
| PET 瓶吸瘪 | 换碳酸饮料瓶；或罐目标真空降到 -60kPa |
| `lgpio.gpiochip_open` 报错 | Pi 5 芯片编号 0/4 随内核版本，`gpioinfo` 确认；用户加入 `gpio` 组或 sudo |
| 泵一直不停 | P1 单传感器下 `read_tank_kpa` 是近似值，滞环失准属预期，台架期泵手动控制 |
| 断电后吸盘马上掉 | 第 3 步的阀型问题（断电通大气），不是漏气；见该步处理 |
| 吸盘在玻璃上打滑 | 唇口或玻璃有灰/油，酒精擦两者；真空度不足 -40kPa |

## 安全

- 12V 回路先断电再改线；XL6009 输出端短路会瞬间烧管。
- 阀/泵是感性负载，确认 MOSFET 板带续流二极管（光耦板一般有，裸管板要自己加）。
- 挂重测试：人不在配重下方，逐级加重，玻璃边缘贴胶带防崩边。
- 泵连续堵转运行会发热，单次测试 ≤5 分钟，摸着烫就歇。
- 锂电池规矩同 P0：充电不离人、XT60 快断随手可拔。
