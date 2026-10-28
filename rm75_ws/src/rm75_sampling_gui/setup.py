#!/usr/bin/env python3
"""Catkin Python package metadata."""

from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


setup_args = generate_distutils_setup(
    packages=[
        "rm75_sampling_gui",
        "rm75_sampling_gui.ros",
        "rm75_sampling_gui.state",
        "rm75_sampling_gui.task",
        "rm75_sampling_gui.ui",
        "rm75_sampling_gui.gui",
        "rm75_sampling_gui.widgets",
        "rm75_sampling_gui.logger",
        "rm75_sampling_gui.model",
        "rm75_sampling_gui.control",
        "rm75_sampling_gui.trajectory",
        "rm75_sampling_gui.web",
        "rm75_sampling_gui.web.backend",
    ],
    # The ROS package directory itself is the Python package; its parent is the
    # distutils package root.
    package_dir={"": ".."},
)

setup(**setup_args)
