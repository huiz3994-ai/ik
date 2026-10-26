#!/usr/bin/env python3

import numpy as np


MODEL = (
    "/root/rm75_ws/ppo_deploy/"
    "rm75_ppo_stage7_v2_200k_numpy.npz"
)


def load_policy():
    d = np.load(MODEL)

    return {
        "w0": d["w0"],
        "b0": d["b0"],
        "w1": d["w1"],
        "b1": d["b1"],
        "wa": d["wa"],
        "ba": d["ba"],
        "low": d["action_low"],
        "high": d["action_high"],
    }


def predict(obs, p):
    x = np.asarray(obs, dtype=np.float32)

    x = np.tanh(
        p["w0"].dot(x) + p["b0"]
    )

    x = np.tanh(
        p["w1"].dot(x) + p["b1"]
    )

    action = (
        p["wa"].dot(x) + p["ba"]
    )

    return np.clip(
        action,
        p["low"],
        p["high"],
    )


def make_obs(q, ee):
    target_relative = np.array(
        [0.08, 0.0, 0.0],
        dtype=np.float32,
    )

    prev_action = np.zeros(
        3,
        dtype=np.float32,
    )

    return np.concatenate(
        [
            q,
            ee,
            target_relative,
            prev_action,
        ]
    ).astype(np.float32)


def main():
    policy = load_policy()

    q_sim = np.array(
        [
            0.0,
            -0.5235988,
            0.0,
            1.0471976,
            0.0,
            0.5235988,
            0.0,
        ],
        dtype=np.float32,
    )

    ee_sim = np.array(
        [
            0.1263895,
            0.0,
            0.7303177,
        ],
        dtype=np.float32,
    )

    q_real = np.array(
        [
            0.055,
            -0.608,
            1.450,
            1.235,
            -2.177,
            -1.472,
            -4.427,
        ],
        dtype=np.float32,
    )

    ee_real = np.array(
        [
            -0.2366,
            0.2449,
            0.3829,
        ],
        dtype=np.float32,
    )

    cases = {
        "A: sim q + sim EE": (
            q_sim,
            ee_sim,
        ),
        "B: real q + real EE": (
            q_real,
            ee_real,
        ),
        "C: real q + sim EE": (
            q_real,
            ee_sim,
        ),
        "D: sim q + real EE": (
            q_sim,
            ee_real,
        ),
    }

    print("=" * 70)
    print("PPO Sim2Real Domain Gap Diagnostic")
    print("Target relative = [+0.08, 0, 0]")
    print("=" * 70)

    for name, (q, ee) in cases.items():
        obs = make_obs(q, ee)
        action = predict(obs, policy)

        print()
        print(name)
        print("q      :", q)
        print("EE     :", ee)
        print("Action :", action)

    print()
    print("=" * 70)


if __name__ == "__main__":
    main()
