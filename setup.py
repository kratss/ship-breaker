#!/usr/bin/env python
from setuptools import setup, find_packages

setup(
    name="ship-cut-planner-lib",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy==2.2.0",
        "pandas==2.2.3 ",
        "matplotlib==3.10.3 ",
        "opencv-python>=4.10,<4.11",
        "open3d>=0.19,<0.20.0 ",
        "PyQt5",
        "icecream",
        "opencv-contrib-python",
        "networkx",
    ],
    author="Jarrett Bolander",
    description="Autonomous ship dismantling path planning library",
    python_requires=">=3.12",
)
