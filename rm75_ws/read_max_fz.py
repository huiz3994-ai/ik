#!/usr/bin/env python3

import rospy
import rostopic


TOPIC_NAME = "/rm_driver/UdpSixZeroForce"

max_fz = None


def callback(msg):
    global max_fz

    try:
        fz = float(msg.force_Fz)
    except AttributeError:
        rospy.logerr("消息中没有 force_Fz 字段")
        rospy.signal_shutdown("Invalid message field")
        return

    # 第一次收到数据
    if max_fz is None:
        max_fz = fz
        print(f"[INIT] force_Fz = {fz:.6f}")
        print(f"[MAX ] force_Fz = {max_fz:.6f}")
        return

    # 出现新的最大值
    if fz > max_fz:
        max_fz = fz
        print(f"[NEW MAX] force_Fz = {max_fz:.6f}")


def main():
    rospy.init_node("read_max_force_fz", anonymous=True)

    print(f"正在读取: {TOPIC_NAME}")
    print("统计 force_Fz 最大值")
    print("Ctrl+C 停止\n")

    # 自动获取 topic 的消息类型
    msg_class, real_topic, _ = rostopic.get_topic_class(
        TOPIC_NAME,
        blocking=True
    )

    if msg_class is None:
        rospy.logerr(f"无法获取 topic 类型: {TOPIC_NAME}")
        return

    print(f"消息类型: {msg_class.__name__}")
    print("--------------------------------")

    rospy.Subscriber(
        TOPIC_NAME,
        msg_class,
        callback,
        queue_size=100
    )

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        print("\n================================")
        if max_fz is not None:
            print(f"force_Fz 最大值 = {max_fz:.6f}")
        else:
            print("没有收到 force_Fz 数据")
        print("================================")


if __name__ == "__main__":
    main()