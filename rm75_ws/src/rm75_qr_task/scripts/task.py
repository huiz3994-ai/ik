#!/usr/bin/env python3
"""Command-line client. PICK/RETURN default to planning preview."""

import argparse
import json
import sys

import rospy

from rm75_qr_task.srv import TaskCommand


def arguments():
    parser = argparse.ArgumentParser(description='RM75 four-marker task client')
    parser.add_argument('command', choices=(
        'pick', 'return', 'preview', 'status', 'stop',
        'recover-holding', 'recover-idle'))
    parser.add_argument('slot_id', type=int, nargs='?')
    parser.add_argument('--execute', action='store_true',
                        help='request execution; server-side authorization and all safety gates still apply')
    parser.add_argument('--i-understand-this-moves-hardware', action='store_true',
                        help='second explicit CLI acknowledgement required with --execute')
    parser.add_argument('--supervised-confirm', action='store_true',
                        help='explicit operator confirmation for a separately enabled debug/recovery path')
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = arguments()
    needs_slot = args.command in ('pick', 'return', 'preview', 'recover-holding')
    if needs_slot and args.slot_id not in (0, 1, 2, 3, 4):
        raise SystemExit('{} requires slot_id 0, 1, 2, 3 or 4'.format(args.command))
    if not needs_slot and args.slot_id is not None:
        raise SystemExit('{} does not accept slot_id'.format(args.command))
    if args.execute != args.i_understand_this_moves_hardware:
        raise SystemExit('real execution requires both --execute and '
                         '--i-understand-this-moves-hardware')
    rospy.init_node('rm75_qr_task_client', anonymous=True)
    rospy.wait_for_service('/qr_task/command', timeout=5.0)
    command = args.command.replace('-', '_')
    response = rospy.ServiceProxy('/qr_task/command', TaskCommand)(
        command, args.slot_id or 0, args.execute, args.supervised_confirm)
    try:
        details = json.loads(response.details_json)
        print(json.dumps(details, indent=2, sort_keys=True))
    except ValueError:
        print(response.details_json)
    if not response.accepted:
        print('REJECTED [{}]: {}'.format(response.state, response.message),
              file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
