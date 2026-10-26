#!/usr/bin/env python3
import json
import os
import tempfile
import unittest

from rm75_qr_task.task_storage import TaskStorage


class StorageTest(unittest.TestCase):
    def test_atomic_save_load_and_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            storage = TaskStorage(os.path.join(directory, 'state.json'))
            state = storage.default_state()
            state['state'] = 'HOLDING'
            storage.save(state)
            self.assertEqual(storage.load()['state'], 'HOLDING')
            archive = storage.archive(state)
            self.assertTrue(os.path.exists(archive))
            with open(archive) as stream:
                self.assertEqual(json.load(stream)['state'], 'HOLDING')
            leftovers = [name for name in os.listdir(directory)
                         if name.startswith('.task-state-')]
            self.assertEqual(leftovers, [])


if __name__ == '__main__':
    unittest.main()
