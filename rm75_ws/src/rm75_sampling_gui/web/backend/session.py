"""Exclusive, short-lived browser control leases.

The ROS joint server intentionally keeps its existing 350 ms heartbeat
watchdog.  A browser lease is shorter than a normal web session and is never
silently renewed by the server: only a live browser WebSocket may renew it.
"""
from dataclasses import dataclass
import secrets
import threading
import time
from typing import Dict, Optional


class SessionError(RuntimeError):
    """Raised when an operation has no valid exclusive browser lease."""


@dataclass
class Lease:
    session_id: str
    client: str
    created_at: float
    expires_at: float
    websocket_id: Optional[str] = None


class ControlSessionManager:
    """Manage one exclusive browser owner with monotonic-time expiry."""

    def __init__(self, timeout_s: float = 0.65) -> None:
        if not 0.4 <= timeout_s <= 2.0:
            raise ValueError("browser control timeout must be between 0.4 and 2 seconds")
        self.timeout_s = float(timeout_s)
        self._lock = threading.RLock()
        self._lease: Optional[Lease] = None

    def _expire_locked(self, now: float) -> None:
        if self._lease is not None and now > self._lease.expires_at:
            self._lease = None

    def acquire(self, client: str) -> Dict[str, object]:
        now = time.monotonic()
        with self._lock:
            self._expire_locked(now)
            if self._lease is not None:
                raise SessionError("控制会话已被其他浏览器占用")
            lease = Lease(secrets.token_urlsafe(24), client[:120], now,
                          now + self.timeout_s)
            self._lease = lease
            return self._public(lease, now)

    def bind_websocket(self, session_id: str, websocket_id: str) -> None:
        now = time.monotonic()
        with self._lock:
            lease = self._require_locked(session_id, now)
            if lease.websocket_id not in (None, websocket_id):
                raise SessionError("控制会话已绑定到另一个连接")
            lease.websocket_id = websocket_id
            lease.expires_at = now + self.timeout_s

    def heartbeat(self, session_id: str, websocket_id: str) -> Dict[str, object]:
        now = time.monotonic()
        with self._lock:
            lease = self._require_locked(session_id, now)
            if lease.websocket_id != websocket_id:
                raise SessionError("心跳连接与控制会话不匹配")
            lease.expires_at = now + self.timeout_s
            return self._public(lease, now)

    def require(self, session_id: str) -> Lease:
        now = time.monotonic()
        with self._lock:
            lease = self._require_locked(session_id, now)
            if lease.websocket_id is None:
                raise SessionError("控制会话尚未建立WebSocket心跳")
            return lease

    def _require_locked(self, session_id: str, now: float) -> Lease:
        self._expire_locked(now)
        if self._lease is None or not secrets.compare_digest(
                self._lease.session_id, str(session_id)):
            raise SessionError("浏览器控制会话无效或已超时")
        return self._lease

    def release(self, session_id: str, websocket_id: Optional[str] = None) -> bool:
        with self._lock:
            lease = self._lease
            if lease is None or not secrets.compare_digest(
                    lease.session_id, str(session_id)):
                return False
            if websocket_id is not None and lease.websocket_id != websocket_id:
                return False
            self._lease = None
            return True

    def snapshot(self) -> Dict[str, object]:
        now = time.monotonic()
        with self._lock:
            self._expire_locked(now)
            if self._lease is None:
                return {"active": False, "remaining_ms": 0}
            result = self._public(self._lease, now)
            result.pop("session_id", None)
            return result

    @staticmethod
    def _public(lease: Lease, now: float) -> Dict[str, object]:
        return {
            "active": True,
            "session_id": lease.session_id,
            "client": lease.client,
            "remaining_ms": max(0, int((lease.expires_at - now) * 1000)),
            "websocket_bound": lease.websocket_id is not None,
        }
