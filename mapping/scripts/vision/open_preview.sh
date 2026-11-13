#!/bin/bash
# Bring this robot's Mac GUI forward, or open the read-only viewer in Terminal.
set -euo pipefail
real="$(cd "$(dirname "$0")/../.." && pwd)"
viewer="$real/scripts/vision/camera_preview.py"
domain="${ROS_DOMAIN_ID:-3}"
if [[ "$domain" != 3 ]]; then
    echo "请在 .103 真机的 ROS Domain 3 环境打开相机预览。" >&2
    exit 1
fi

preview_pid() {
    local pid
    while read -r pid; do
        if /bin/ps eww -p "$pid" -o command= | /usr/bin/awk -v domain="$domain" '
            { for (i = 1; i <= NF; i++) if ($i == "ROS_DOMAIN_ID=" domain) found = 1 }
            END { exit !found }'; then
            echo "$pid"
            return 0
        fi
    done < <(/bin/ps -axo pid=,command= | /usr/bin/awk -v script="$viewer" '
        $2 ~ /(^|\/)([Pp]ython)([0-9.]+)?$/ && substr($0,length($0)-length(script)+1) == script { print $1 }')
    return 1
}

raise_preview() {
    /usr/bin/osascript -l JavaScript - "$1" <<'JXA'
ObjC.import("AppKit");
ObjC.import("CoreGraphics");
function run(args) {
    const pid = Number(args[0]);
    const app = $.NSRunningApplication.runningApplicationWithProcessIdentifier(pid);
    if (app.isHidden) app.unhide;
    app.activateWithOptions(3);
    const windows = ObjC.deepUnwrap(ObjC.castRefToObject($.CGWindowListCopyWindowInfo(1, 0)));
    const view = windows.find(w => w.kCGWindowOwnerPID === pid && w.kCGWindowLayer === 0);
    if (!view) throw Error("预览窗口尚未出现");
    return (view.kCGWindowName || "TurtleBot .103 camera") + " (PID " + pid + ")";
}
JXA
}

if ! pid="$(preview_pid)"; then
    /usr/bin/open -g -a Terminal "$real/打开真机相机.command"
fi
for ((attempt=0; attempt<30; attempt++)); do
    if pid="$(preview_pid)" && raise_preview "$pid" 2>/dev/null; then
        exit 0
    fi
    sleep 0.4
done
echo "相机预览窗口尚未显示。请查看刚打开的终端或 camera-preview.log。" >&2
exit 1
