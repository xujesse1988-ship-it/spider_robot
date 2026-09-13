#!/usr/bin/env bash
# 树莓派 → 开发机 黑匣子日志同步（09-13）。cron 每分钟起一次，本脚本在这一分钟里
# 每 10s 推一次，只推文件名日期是**今天**的 logs/<tag>_YYYYmmdd_HHMMSS.log（按树莓派
# 本地日期，和 runlog 起名同口径），昨天及更早的一个不碰。用 rsync：没变的文件不传、
# 正在写的文件只传新增部分、开发机上永远是完整快照（rsync 先写临时文件再改名）。
#
# 装（在树莓派上，一次）：
#   1. ssh -o BatchMode=yes shaopeng@64.176.60.202 true     # 没报错 = 免密 OK
#   2. crontab -e 加一行：
#      * * * * * $HOME/spider_robot/software/scripts/pi_sync_logs.sh >> /tmp/pi_sync_logs.log 2>&1
#   看有没有失败：tail /tmp/pi_sync_logs.log      # 只记失败，空的就是一直成功
#   停：crontab -e 把那行删掉
#
# 网络断了不会堆进程：单次 rsync 25s 超时；上一分钟的实例还没退（网络卡住）时，
# 这一分钟的最多等 30s 接棒，接不到就静默退出（flock）。
#
# 测试用环境变量：SRC / DST / DURATION（默认 55s；0 = 只推一次就退）。

set -u
SRC="${SRC:-$HOME/spider_robot/software/logs}"
DST="${DST:-shaopeng@64.176.60.202:spider/software/logs/}"
DURATION="${DURATION:-55}"
INTERVAL=10

[[ -d "$SRC" ]] || { echo "$(date '+%F %T') 没有日志目录 $SRC"; exit 1; }

exec 9>/tmp/pi_sync_logs.lock
flock -w 30 9 || exit 0

sync_once() {
    local today out rc
    today=$(date +%Y%m%d)
    out=$(timeout 25 rsync -az --include="*_${today}_*.log" --exclude='*' \
            -e 'ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new' \
            "$SRC/" "$DST" 2>&1)
    rc=$?
    (( rc == 0 )) || echo "$(date '+%F %T') 同步失败 rc=$rc ${out//$'\n'/ | }"
}

# SECONDS 从脚本启动计，等锁的时间也算在窗口里，不会拖过下一分钟
while :; do
    sync_once
    (( SECONDS + INTERVAL >= DURATION )) && break
    sleep "$INTERVAL"
done
exit 0
