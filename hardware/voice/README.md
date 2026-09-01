# hardware/voice —— ReSpeaker 2-Mics Pi HAT 的设备树覆盖层

`respeaker-2mic-v1_0-overlay.dts` 逐字节来自 Seeed 官方仓库
<https://github.com/Seeed-Studio/seeed-linux-dtoverlays>（`overlays/rpi/`，MIT License，
文件头版权 2026），随仓库放一份是为了树莓派上**不用 clone 那个仓库、不用 make**，
一条 `dtc` 就能编：

```bash
sudo apt install -y device-tree-compiler
dtc -@ -I dts -O dtb -o respeaker-2mic-v1_0.dtbo hardware/voice/respeaker-2mic-v1_0-overlay.dts
sudo cp respeaker-2mic-v1_0.dtbo /boot/firmware/overlays/
echo dtoverlay=respeaker-2mic-v1_0 | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

（`software/scripts/voice_setup.sh` 第 2 步就是这几行。）

## 为什么选它

- **纯设备树**，不带内核模块：用的是树莓派内核自带的 `snd-soc-wm8960` 和
  `snd-soc-simple-card`（rpi-6.6.y / rpi-6.12.y 的 `bcm2712_defconfig` 里
  `CONFIG_SND_SOC_WM8960=m`、`CONFIG_SND_SIMPLE_CARD=m` 已核对），内核升级不用重编。
- `compatible` 里列了 `brcm,bcm2712`（Pi 5），I2S 挂 `&i2s_clk_consumer`
  （WM8960 用板上 24 MHz 晶振做位时钟/帧时钟主机，Pi 做从机），
  `wm8960_mclk` 固定时钟 24 MHz——和这块板的硬件一致。
- 声卡名 `seeed2micvoicec`，`software/hexapod/voice/audio.py` 按这个名字找卡。

## 不选另外两条路的原因

| 方案 | 问题 |
|---|---|
| 树莓派固件内置 `dtoverlay=wm8960-soundcard` | 是给 Waveshare WM8960 板写的：Pi 做时钟主机（`i2s_clk_producer`）、`wm8960_mclk` 按 **12.288 MHz** 声明；ReSpeaker 板上是 24 MHz 晶振且设计为编解码器主时钟，驱动算出的分频会错 |
| `respeaker/seeed-voicecard` / `HinTak/seeed-voicecard`（dkms 内核模块） | 每个内核版本一个分支、随系统升级重编、官方只认 Pi 3/4；Seeed 自己的新文档已改推 dtoverlay 方案 |

## 板卡引脚（接线前必读）

见 `docs/VOICE-GUIDE.md` §1：I2S 固定用 GPIO18/19/20/21，其中 19/20/21 和现有
气路（阀 R2 / 泵 A / 阀 R3）冲突要挪；GPIO17（HAT 按键，板上有外接上拉）与舵机
继电器冲突**绝不能接**；GPIO5（HAT 灯电源）与阀 L1 冲突不接。
