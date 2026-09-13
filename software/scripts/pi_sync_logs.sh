#!/usr/bin/env bash
# 树莓派 → 开发机 黑匣子日志同步（09-13），在树莓派上手工跑：
#   ~/spider_robot/software/scripts/pi_sync_logs.sh            # 推今天的
#   ~/spider_robot/software/scripts/pi_sync_logs.sh 20260912   # 推指定日期的
#                                （跨午夜的那次运行，文件名日期是前一天，用这个补）
# 只推文件名日期匹配的 logs/<tag>_YYYYmmdd_HHMMSS.log（按树莓派本地日期，和 runlog
# 起名同口径），其余不碰。rsync：没变的不传、正在写的只传新增部分。
# 推到 shaopeng@64.176.60.202:~/spider/software/logs/，免密前提是树莓派的
# id_ed25519.pub 在开发机 authorized_keys 里（09-13 已放）。
# 测试用环境变量：SRC / DST。

set -u
SRC="${SRC:-$HOME/spider_robot/software/logs}"
DST="${DST:-shaopeng@64.176.60.202:spider/software/logs/}"
day="${1:-$(date +%Y%m%d)}"

[[ -d "$SRC" ]] || { echo "没有日志目录 $SRC" >&2; exit 1; }
[[ "$day" =~ ^[0-9]{8}$ ]] || { echo "日期要写成 YYYYmmdd，收到 '$day'" >&2; exit 2; }

out=$(rsync -az --info=NAME --include="*_${day}_*.log" --exclude='*' \
        -e 'ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new' \
        "$SRC/" "$DST" 2>&1)
rc=$?
if (( rc != 0 )); then
    echo "$out" >&2
    echo "同步失败 rc=$rc" >&2
    exit "$rc"
fi
files=$(printf '%s\n' "$out" | grep -v '^\./\?$' | grep .)
if [[ -z "$files" ]]; then
    echo "$day：开发机已是最新，没东西要推"
else
    echo "$files"
    echo "推了 $(printf '%s\n' "$files" | wc -l) 个文件 → $DST"
fi
