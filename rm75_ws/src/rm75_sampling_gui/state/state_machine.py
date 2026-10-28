"""Pure-Python state machine for the RM75 sampling HMI.

This module deliberately contains no ROS publishers and no robot motion logic.
It only models operator workflow state and provides transition notifications.
"""

from dataclasses import dataclass
from enum import Enum
import logging
import threading
from typing import Callable, List, Optional


LOGGER = logging.getLogger(__name__)


class RobotState(str, Enum):
    """Top-level operator states shown by the HMI."""

    IDLE = "IDLE"
    TELEOP = "TELEOP"
    TOOL_DETECT = "TOOL_DETECT"
    TOOL_PICK = "TOOL_PICK"
    TOOL_RETURN = "TOOL_RETURN"
    TOOL_PLACE = "TOOL_PLACE"
    FORCE_CONTROL = "FORCE_CONTROL"
    JOINT_CONTROL = "JOINT_CONTROL"
    RECOVERY = "RECOVERY"
    SAFE_STOP = "SAFE_STOP"


class StateTransitionError(RuntimeError):
    """Raised when an invalid state transition is requested."""


@dataclass(frozen=True)
class StateTransition:
    """Immutable transition record delivered to listeners."""

    previous: RobotState
    current: RobotState
    reason: str


StateListener = Callable[[StateTransition], None]


class StateMachine:
    """Thread-safe operation status with a latched fail-safe state.

    Normal states describe the currently selected function; they are not a
    workflow gate.  Only SAFE_STOP is restrictive and must pass through the
    non-motion RECOVERY operation before another function can be selected.
    """

    def __init__(self, initial_state: RobotState = RobotState.IDLE) -> None:
        self._state = initial_state
        self._listeners: List[StateListener] = []
        self._lock = threading.RLock()
        LOGGER.info("状态机初始化：%s", initial_state.value)

    @property
    def state(self) -> RobotState:
        """Return the current state."""

        with self._lock:
            return self._state

    def add_listener(self, listener: StateListener) -> None:
        """Register a callback invoked after every successful transition."""

        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    def remove_listener(self, listener: StateListener) -> None:
        """Remove a previously registered callback."""

        with self._lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def can_transition(self, target: RobotState) -> bool:
        """Return whether ``target`` is valid from the current state."""

        with self._lock:
            return (
                target == self._state
                or self._state != RobotState.SAFE_STOP
                or target == RobotState.RECOVERY
            )

    def transition(self, target: RobotState, reason: str = "operator request") -> StateTransition:
        """Move to ``target`` and notify listeners.

        A transition to the current state is treated as an idempotent no-op and
        still returns a transition record. Invalid transitions raise
        :class:`StateTransitionError`.
        """

        if not isinstance(target, RobotState):
            raise TypeError("target must be a RobotState")

        normalized_reason = reason.strip() or "unspecified"
        with self._lock:
            previous = self._state
            if (
                target != previous
                and previous == RobotState.SAFE_STOP
                and target != RobotState.RECOVERY
            ):
                raise StateTransitionError(
                    "SAFE_STOP已锁存，必须先执行恢复检查：{} -> {}".format(
                        previous.value, target.value
                    )
                )
            self._state = target
            listeners = list(self._listeners)

        transition = StateTransition(previous, target, normalized_reason)
        if previous == target:
            LOGGER.debug("状态保持 %s：%s", target.value, normalized_reason)
        else:
            LOGGER.info(
                "状态切换 %s -> %s：%s",
                previous.value,
                target.value,
                normalized_reason,
            )
        for listener in listeners:
            try:
                listener(transition)
            except Exception:
                LOGGER.exception("状态监听器执行失败")
        return transition

    def fail(self, error: BaseException, context: Optional[str] = None) -> StateTransition:
        """Latch the state machine into ``SAFE_STOP`` after an exception."""

        detail = "{}: {}".format(context, error) if context else str(error)
        with self._lock:
            previous = self._state
            self._state = RobotState.SAFE_STOP
            listeners = list(self._listeners)

        transition = StateTransition(previous, RobotState.SAFE_STOP, detail)
        LOGGER.error(
            "异常触发 SAFE_STOP，原状态=%s，原因=%s", previous.value, detail
        )
        for listener in listeners:
            try:
                listener(transition)
            except Exception:
                LOGGER.exception("SAFE_STOP 状态监听器执行失败")
        return transition
