#!/usr/bin/env bash
# 树莓派死机验尸工具（09-02：body_lean/climb_walk 启动时阀线圈逐一通电后概率性
# 整机死机，灯绿→红、SSH 失联；09-04 walk_teleop 启动也死过）。两条子命令：
#
#   setup [--ramoops] [--watchdog]   一次性准备（死机**之前**做，否则内核日志随
#                                    死机蒸发）：journal 落盘持久化 + 1s 同步；
#                                    --ramoops 加 dtoverlay=ramoops（内核 panic
#                                    文本跨重启保留，需重启生效）；--watchdog 让
#                                    systemd 喂硬件看门狗（内核僵死 15s 自动重启；
#                                    PMIC 已把 SoC 断电的红灯态它救不了）
#   check                            重新上电后跑：上次开机的内核电源/崩溃线索、
#                                    pstore、当前欠压标志/PMIC 电压、黑匣子尾巴
#
# 怎么读 check 的输出（对应 hexapod/powerlog.py 写的黑匣子行）：
#   * 黑匣子最后一行停在 "启动 阀 k/6 线圈通电前" → 第 k 路线圈通电是扳机；
#     停在 "舵机继电器已合闸"/"合闸后 0.05s" → 18 舵机同刻上电的冲击是扳机；
#     --startup-gap 3 把两件事隔开 3s 再复现一次，归属就没有歧义
#   * 最后几行 "TLM 电源 5V=…" 若一路下探到 4.8 以下、或出现 "曾欠压" 粘滞位
#     → 5V 降压输入被电池塌陷拖垮（PMIC 欠压停机=红灯），治供电；
#     5V 一直 5.0x 平稳、内核也没有 voltage 字样却直接死 → 偏向干扰/地弹
#     （续流二极管、地线拓扑、I2C/USB 线离 12V 束）或 SoC/PCIe 侧问题
#     ——09-04 定案就是这一种：合闸充电脉冲经 Pi↔Servo2040 USB 地环路进 Pi，
#     拔 USB 合闸十次 0x0、插回第一次 0x50000；案卷 html/pi-crash-ground-loop-20260904.html
#   * journalctl -b -1 末尾有 "Undervoltage detected" / hwmon 字样 = 内核自己
#     也看见了欠压；有 Oops/panic = 软件崩溃（看 pstore 全文）；什么都没有、
#     日志戛然而止 = 断电式死亡（PMIC 停机）最典型的样子
#   * /proc/device-tree/chosen/power/power_reset：树莓派工程师 timg236 论坛原话
#     说非零（通常 2）= 上次开机被 PMIC 因低压断电（forums.raspberrypi.com
#     t=361231，前提是用电源键重启且没断过电）。09-03/09-04 两次死机后读到 2，
#     **但 09-06 对照：正常 poweroff/reboot 几次之后仍读 2**——本机上此字段没有
#     区分力，不能当证据，check 只是照抄一份。死后红灯常亮本身已说明 SoC 被
#     PMIC 断电（内核崩溃灯不会变红），不需要它
#   * 黑匣子标签：climb_（climb_walk）/lean_（body_lean）/walk_（walk_teleop）/
#     voice_（voice_teleop），check 只看最新一份
set -u
here=$(cd "$(dirname "$0")" && pwd)
logs="$here/../logs"
cmd=${1:-check}; shift || true

boot_cfg() {
    for f in /boot/firmware/config.txt /boot/config.txt; do
        [ -f "$f" ] && { echo "$f"; return; }
    done
}

do_setup() {
    local ramoops=0 watchdog=0
    for a in "$@"; do
        case "$a" in
            --ramoops) ramoops=1 ;;
            --watchdog) watchdog=1 ;;
            *) echo "未知参数 $a"; exit 2 ;;
        esac
    done
    echo "== journal 持久化（内核日志落盘，死机后 journalctl -b -1 才有东西）"
    sudo mkdir -p /var/log/journal /etc/systemd/journald.conf.d
    printf '[Journal]\nStorage=persistent\nSyncIntervalSec=1s\n' \
        | sudo tee /etc/systemd/journald.conf.d/forensics.conf >/dev/null
    sudo systemd-tmpfiles --create --prefix /var/log/journal 2>/dev/null || true
    sudo systemctl restart systemd-journald && echo "   已开：Storage=persistent SyncIntervalSec=1s"
    if [ "$ramoops" = 1 ]; then
        cfg=$(boot_cfg)
        if ls /boot/firmware/overlays/ramoops.dtbo /boot/overlays/ramoops.dtbo >/dev/null 2>&1; then
            if grep -q '^dtoverlay=ramoops' "$cfg"; then
                echo "== ramoops 已在 $cfg"
            else
                echo 'dtoverlay=ramoops' | sudo tee -a "$cfg" >/dev/null
                echo "== 已追加 dtoverlay=ramoops 到 $cfg（重启后生效；panic 文本落 /sys/fs/pstore → systemd-pstore 搬到 /var/lib/systemd/pstore）"
            fi
        else
            echo "== ⚠ 没找到 ramoops.dtbo 覆盖层，跳过"
        fi
    fi
    if [ "$watchdog" = 1 ]; then
        sudo mkdir -p /etc/systemd/system.conf.d
        printf '[Manager]\nRuntimeWatchdogSec=15\nShutdownWatchdogSec=2min\n' \
            | sudo tee /etc/systemd/system.conf.d/watchdog.conf >/dev/null
        sudo systemctl daemon-reexec && echo "== 硬件看门狗：内核 15s 不喂即重启（只对内核僵死有效）"
    fi
    echo "== 当前用户在 video 组？（vcgencmd pmic_read_adc 需要）"
    id -nG | tr ' ' '\n' | grep -qx video && echo "   是" || echo "   ⚠ 否：sudo usermod -aG video $USER 后重新登录"
    echo "完成。复现死机 → 重新上电 → $0 check"
}

do_check() {
    echo "===== 开机列表（-1 = 上一次开机，即死机那次）"
    journalctl --list-boots --no-pager 2>/dev/null | tail -n 3 || echo "（journalctl 不可用）"
    [ -d /var/log/journal ] && echo "journal: persistent" || echo "⚠ journal 未持久化：上次开机的内核日志已丢，先跑 $0 setup"
    echo
    echo "===== 上次开机 内核里的电源/崩溃字样（空=内核没来得及说话）"
    journalctl -b -1 -k --no-pager 2>/dev/null \
        | grep -iE 'volt|pmic|hwmon|throttl|oops|panic|BUG:|watchdog|brown|reset' | tail -n 30
    echo
    echo "===== 上次开机 最后 25 行（死前系统在干什么；戛然而止=断电式死亡）"
    journalctl -b -1 --no-pager -o short-monotonic 2>/dev/null | tail -n 25
    echo
    echo "===== 上次开机 真实时间线（Pi 5 无 RTC 电池：开机列表里的起始时刻是上次存盘的假"
    echo "      时钟，联网校时后才跳到真时间；用最后一条的真时间倒推开机时刻）"
    last_iso=$(journalctl -b -1 -n 1 --no-pager -o short-iso 2>/dev/null | awk '{print $1}')
    last_mono=$(journalctl -b -1 -n 1 --no-pager -o short-monotonic 2>/dev/null \
                | sed -n 's/^\[ *\([0-9.]*\)\].*/\1/p')
    if [ -n "$last_iso" ] && [ -n "$last_mono" ]; then
        last_epoch=$(date -d "$last_iso" +%s 2>/dev/null)
        if [ -n "$last_epoch" ]; then
            boot_epoch=$(awk -v e="$last_epoch" -v m="$last_mono" 'BEGIN{printf "%d", e - m}')
            echo "  最后一条日志: $last_iso（开机后 ${last_mono}s）"
            echo "  倒推真实开机时刻: $(date -d @"$boot_epoch" '+%F %T')；此后到死机之间系统没再写日志"
            echo "  （死机时刻 ≥ 最后一条；黑匣子行内的 uptime/时间戳可与之对齐）"
        fi
    else
        echo "  （journalctl -b -1 无内容）"
    fi
    echo
    echo "===== 关机/重启记录（wtmp）：只有 reboot 没有 shutdown = 非正常断电"
    last -x -n 6 shutdown reboot 2>/dev/null | head -n 8
    echo
    echo "===== pstore（内核 panic 文本，需 setup --ramoops 且重启过）"
    ls -la /sys/fs/pstore /var/lib/systemd/pstore 2>/dev/null | grep -v '^total' || echo "（空）"
    echo
    echo "===== 现在的电源状态（get_throttled 粘滞位在重启后清零，只能反映本次开机）"
    command -v vcgencmd >/dev/null && {
        vcgencmd get_throttled
        vcgencmd pmic_read_adc 2>/dev/null | grep -E 'EXT5V|VDD_CORE|3V3_SYS' || echo "（pmic_read_adc 不可用：非 Pi 5 或不在 video 组）"
    } || echo "（无 vcgencmd）"
    for n in /sys/class/hwmon/hwmon*; do
        [ -f "$n/name" ] && [ "$(cat "$n/name")" = rpi_volt ] && \
            echo "hwmon rpi_volt in0_lcrit_alarm=$(cat "$n/in0_lcrit_alarm" 2>/dev/null)"
    done
    echo
    echo "===== 固件电源节点 /proc/device-tree/chosen/power（大端 u32；power_reset 09-06 对照发现"
    echo "      正常关机后也读 2，本机无区分力、仅记录；max_current 单位 mA）"
    for f in /proc/device-tree/chosen/power/*; do
        [ -f "$f" ] && printf '%s = ' "$(basename "$f")" && od -An -tx1 "$f" | tr -s ' \n' ' ' && echo
    done 2>/dev/null
    echo
    echo "===== config.txt / EEPROM 里与供电相关的行"
    cfg=$(boot_cfg); [ -n "${cfg:-}" ] && grep -nE 'usb_max_current|psu_max_current|ramoops|watchdog' "$cfg"
    command -v rpi-eeprom-config >/dev/null && rpi-eeprom-config 2>/dev/null | grep -E 'POWER_OFF_ON_HALT|PSU_MAX_CURRENT|WAKE_ON_GPIO'
    echo
    echo "===== 黑匣子尾巴（最新一份 climb_/lean_/walk_/voice_ 日志；看最后一行停在哪一步、5V 趋势）"
    latest=$(ls -t "$logs"/*_*.log 2>/dev/null | head -n 1)
    if [ -n "$latest" ]; then
        echo "$latest"
        grep -m1 '^# boot_id' "$latest"
        grep -m1 '^# Pi 电源监视' "$latest"
        tail -n 20 "$latest"
    else
        echo "（$logs 下没有日志）"
    fi
}

case "$cmd" in
    setup) do_setup "$@" ;;
    check) do_check ;;
    -h|--help|help) sed -n '2,25p' "$0" ;;
    *) echo "用法: $0 setup [--ramoops] [--watchdog] | check"; exit 2 ;;
esac
