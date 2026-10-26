"""Asynchronous experiment CSV and task-log recording, never a control source."""
import copy
import csv
from datetime import datetime, timezone
import logging
from pathlib import Path
import queue
import threading
from typing import Any, Dict
import yaml
from rm75_sampling_gui.state.state_machine import RobotState, StateTransition


class ExperimentLogger(logging.Handler):
    """One ordered writer thread; bounded queue and visible recording failures."""

    ROBOT_COLUMNS = (["timestamp"] + ["joint{}".format(i) for i in range(1, 8)] +
                     ["x", "y", "z", "rx", "ry", "rz", "joint_source_timestamp",
                      "tf_source_timestamp", "joint_fresh", "tcp_fresh"])
    FORCE_COLUMNS = ["timestamp", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz",
                     "force_z", "filtered_force_z", "force_setpoint",
                     "wrench_fresh", "force_fresh", "wrench_source_timestamp"]

    def __init__(self, root: str, config: Dict[str, Any]) -> None:
        super().__init__(logging.DEBUG)
        self.root = Path(root).expanduser().resolve()
        self.config = copy.deepcopy(config)
        self.queue = queue.Queue(maxsize=int(config.get("experiment", {}).get("queue_size", 2048)))
        self.error = ""
        self.directory = None
        self.active = False
        self.closed_writer = False
        self._streams = {}
        self._writers = {}
        self._worker = threading.Thread(target=self._run, name="experiment-writer", daemon=True)
        self._worker.start()

    def _enqueue(self, kind: str, value: Any) -> None:
        if self.closed_writer:
            return
        try:
            self.queue.put_nowait((kind, value))
        except queue.Full:
            self.error = "实验记录队列已满，数据可能丢失"
            # Exposed to the GUI rather than logging recursively from a Handler.

    def emit(self, record: logging.LogRecord) -> None:
        self._enqueue("event", "{} [{}] {}: {}".format(
            datetime.fromtimestamp(record.created, timezone.utc).isoformat(),
            record.levelname, record.name, record.getMessage()))

    def transition(self, event: StateTransition) -> None:
        self._enqueue("transition", event)

    def begin_task(self) -> None:
        self._enqueue("begin", None)

    def sample(self, snapshot: Dict[str, Any]) -> None:
        # Never queue images or unbounded ROS message objects.
        keys = ("timestamp", "joints", "tcp", "wrench", "fresh", "source_stamps",
                "force_z", "filtered_force_z", "force_setpoint")
        self._enqueue("sample", copy.deepcopy({key: snapshot.get(key) for key in keys}))

    def _begin(self) -> None:
        if self.active:
            return
        self.root.mkdir(parents=True, exist_ok=True)
        day = datetime.now().strftime("%Y%m%d")
        index = 1
        while True:
            directory = self.root / "{}_{:03d}".format(day, index)
            try:
                directory.mkdir()
                break
            except FileExistsError:
                index += 1
        self.directory = directory
        try:
            with (directory / "config.yaml").open("x", encoding="utf-8") as stream:
                yaml.safe_dump({"parameters": self.config,
                                "recording": {"joint_unit": "rad", "tcp_position_unit": "m",
                                              "tcp_rotation": "RPY radians",
                                              "wrench_source": self.config.get("monitoring", {}).get("wrench_topic"),
                                              "wrench_timestamp": "receipt (Six_Force has no header)",
                                              "timestamp": "UTC Unix seconds (receipt/snapshot)",
                                              "missing_values": "empty CSV cells"}},
                               stream, allow_unicode=True, sort_keys=False)
            for name, columns in (("robot_state.csv", self.ROBOT_COLUMNS), ("force.csv", self.FORCE_COLUMNS)):
                stream = (directory / name).open("x", encoding="utf-8", newline="")
                self._streams[name] = stream
                self._writers[name] = csv.writer(stream)
                self._writers[name].writerow(columns)
            self._streams["task.log"] = (directory / "task.log").open("x", encoding="utf-8")
            self.active = True
            self._event("EXPERIMENT START " + directory.name)
        except Exception:
            self._finish()
            raise

    def _event(self, text: str) -> None:
        if self.active:
            self._streams["task.log"].write(text + "\n")
        elif self.directory is not None:
            # Late ACK/error results after SAFE_STOP remain in the same experiment.
            with (self.directory / "task.log").open("a", encoding="utf-8") as stream:
                stream.write(text + "\n")

    def _transition(self, event: StateTransition) -> None:
        if not self.active and event.current in (
                RobotState.TELEOP, RobotState.TOOL_DETECT, RobotState.TOOL_PICK,
                RobotState.TOOL_HOLD, RobotState.FORCE_CONTROL):
            self._begin()
        self._event("{} STATE {} -> {}: {}".format(
            datetime.now(timezone.utc).isoformat(), event.previous.value,
            event.current.value, event.reason))
        if event.current in (RobotState.IDLE, RobotState.SAFE_STOP):
            self._finish()

    def _sample(self, data: Dict[str, Any]) -> None:
        if not self.active:
            return
        fresh, stamps = data.get("fresh") or {}, data.get("source_stamps") or {}
        joints = data.get("joints") or {} if fresh.get("joints") else {}
        tcp = data.get("tcp") or {} if fresh.get("tcp") else {}
        wrench = data.get("wrench") or {} if fresh.get("wrench") else {}
        self._writers["robot_state.csv"].writerow(
            [data.get("timestamp")] + [joints.get("joint{}".format(i), "") for i in range(1, 8)] +
            [tcp.get(key, "") for key in ("x", "y", "z", "rx", "ry", "rz")] +
            [stamps.get("joints", ""), stamps.get("tcp", ""), bool(fresh.get("joints")), bool(fresh.get("tcp"))])
        self._writers["force.csv"].writerow(
            [data.get("timestamp")] + [wrench.get(key, "") for key in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")] +
            [data.get(key) if fresh.get(key) else "" for key in ("force_z", "filtered_force_z", "force_setpoint")] +
            [bool(fresh.get("wrench")), bool(fresh.get("force_z") or fresh.get("filtered_force_z")),
             stamps.get("wrench", "")])

    def _finish(self) -> None:
        if self.active:
            self._event("EXPERIMENT END")
        for stream in self._streams.values():
            stream.flush()
            stream.close()
        self._streams.clear()
        self._writers.clear()
        self.active = False

    def _run(self) -> None:
        while True:
            kind, value = self.queue.get()
            try:
                if kind == "shutdown":
                    self._finish()
                    return
                if kind == "begin":
                    self._begin()
                elif kind == "transition":
                    self._transition(value)
                elif kind == "sample":
                    self._sample(value)
                elif kind == "event":
                    self._event(value)
                for stream in self._streams.values():
                    stream.flush()
            except Exception as error:
                self.error = "实验记录失败：{}".format(error)
                for stream in self._streams.values():
                    try:
                        stream.close()
                    except OSError:
                        pass
                self._streams.clear()
                self.active = False
            finally:
                self.queue.task_done()

    def flush_pending(self) -> None:
        self.queue.join()

    def close(self) -> None:
        if not self.closed_writer:
            self.closed_writer = True
            self.queue.put(("shutdown", None))
            self._worker.join(timeout=5.0)
            if self._worker.is_alive():
                self.error = "实验记录关闭超时"
        super().close()
