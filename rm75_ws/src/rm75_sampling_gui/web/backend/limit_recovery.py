"""Fixed, bounded web adapter for the RealMan soft-limit recovery tool."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import threading
from typing import Any, Dict


_DEFAULT_SCRIPT = Path(__file__).resolve().parents[5] / "realman_limit_recovery.py"
_RUN_LOCK = threading.Lock()


def script_path() -> Path:
    """Return the configured recovery script without accepting a web-supplied path."""
    return Path(os.environ.get("RM75_LIMIT_RECOVERY_SCRIPT", str(_DEFAULT_SCRIPT))).resolve()


def connection_target() -> Dict[str, Any]:
    """Expose the same controller target inherited by the recovery script."""
    host = os.environ.get("RM75_ARM_IP", "192.168.1.18")
    try:
        port = int(os.environ.get("RM75_ARM_PORT", "8080"))
    except ValueError as error:
        raise RuntimeError("RM75_ARM_PORT 必须是整数") from error
    if not host or not 1 <= port <= 65535:
        raise RuntimeError("机械臂控制器地址配置无效")
    return {"host": host, "port": port, "target": f"{host}:{port}"}


def run_limit_recovery(recover: bool = False) -> Dict[str, Any]:
    """Run diagnosis or the explicitly confirmed recovery command.

    Diagnosis exit code 2 means that the script found joints near a soft limit;
    it is a successful diagnostic outcome rather than a transport failure.
    """
    path = script_path()
    target = connection_target()
    if not path.is_file():
        raise RuntimeError(f"限位工具不存在：{path}")
    if not _RUN_LOCK.acquire(blocking=False):
        raise RuntimeError("已有一个限位查询或修复正在执行，请等待完成")

    argv = [sys.executable, str(path)]
    if recover:
        argv.extend(("--recover", "--yes"))
    try:
        result = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=75.0 if recover else 20.0,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            "限位修复等待超时；请立即检查机械臂状态"
            if recover else "限位查询超时；请检查控制器网络连接"
        ) from error
    except OSError as error:
        raise RuntimeError(f"无法启动限位工具：{error}") from error
    finally:
        _RUN_LOCK.release()

    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    output = "\n".join(part for part in (stdout, stderr) if part)
    accepted_codes = {0} if recover else {0, 2}
    if result.returncode not in accepted_codes:
        raise RuntimeError(output or f"限位工具执行失败（退出码 {result.returncode}）")
    return {
        "ok": result.returncode == 0,
        "needs_recovery": not recover and result.returncode == 2,
        "recovered": recover and result.returncode == 0,
        "exit_code": result.returncode,
        "command": " ".join(str(item) for item in argv[1:]),
        **target,
        "output": output or "命令已完成，但没有输出。",
    }
