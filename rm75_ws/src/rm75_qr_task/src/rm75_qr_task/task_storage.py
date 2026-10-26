"""Crash-resistant JSON persistence for task state and archived records."""

import json
import os
import tempfile
import time


class TaskStorage(object):
    def __init__(self, path, archive_dir=None):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.archive_dir = os.path.abspath(os.path.expanduser(
            archive_dir or os.path.join(os.path.dirname(self.path), 'archive')))

    @staticmethod
    def default_state():
        return {
            'schema_version': 1,
            'state': 'IDLE',
            'held_slot_id': None,
            'last_successful_action': None,
            'pending_action': None,
            'last_error': '',
            'power_commanded_voltage': None,
            'hold_confirmed': False,
            'record': None,
            'station_records': {},
            'updated_at': time.time(),
        }

    def load(self):
        if not os.path.exists(self.path):
            return self.default_state()
        with open(self.path, 'r') as stream:
            state = json.load(stream)
        if state.get('schema_version') != 1 or 'state' not in state:
            raise ValueError('unsupported or corrupt task state')
        return state

    def save(self, state):
        parent = os.path.dirname(self.path)
        os.makedirs(parent, exist_ok=True)
        value = dict(state)
        value['schema_version'] = 1
        value['updated_at'] = time.time()
        fd, temporary = tempfile.mkstemp(prefix='.task-state-', dir=parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(value, stream, indent=2, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory_fd = os.open(parent, os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def archive(self, state):
        os.makedirs(self.archive_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime())
        destination = os.path.join(self.archive_dir, 'task-{}.json'.format(stamp))
        suffix = 0
        while os.path.exists(destination):
            suffix += 1
            destination = os.path.join(
                self.archive_dir, 'task-{}-{}.json'.format(stamp, suffix))
        with open(destination, 'w') as stream:
            json.dump(state, stream, indent=2, sort_keys=True)
            stream.write('\n')
        return destination
