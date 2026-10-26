"""Independent authorization, commissioning, and runtime gates."""


class SafetyError(RuntimeError):
    pass


REQUIRED_VERIFICATIONS = (
    'marker_geometry_verified',
    'hand_eye_calibrated',
    'tcp_verified',
    'tool_geometry_verified',
    'object_geometry_verified',
    'payload_verified',
    'hold_feedback_verified',
    'voltage_interface_verified',
    'collision_scene_verified',
    'execution_monitor_verified',
    'stop_verified',
    'single_command_source_verified',
)


class SafetyChecker(object):
    def __init__(self, execution_authorized=False, simulation=False,
                 verifications=None, runtime_check=None):
        self.execution_authorized = bool(execution_authorized)
        self.simulation = bool(simulation)
        self.verifications = dict(verifications or {})
        self.runtime_check = runtime_check or (lambda: (True, ''))

    def check_preview(self):
        ok, reason = self.runtime_check()
        if not ok:
            raise SafetyError(reason or 'runtime safety check failed')

    def check_execute(self, request_execute):
        if not request_execute:
            raise SafetyError('request did not explicitly authorize execution')
        if not self.execution_authorized:
            raise SafetyError('server execution_authorized is false')
        missing = [name for name in REQUIRED_VERIFICATIONS
                   if not self.verifications.get(name, False)]
        if missing:
            raise SafetyError('hardware readiness incomplete: ' + ', '.join(missing))
        self.check_preview()
