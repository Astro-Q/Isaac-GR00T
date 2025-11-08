#!/usr/bin/env python3
"""
Setup script for Unitree G1 + GR00T-N1.5 Integration
"""

from setuptools import setup, find_packages
import os

# Read README
readme_path = os.path.join(os.path.dirname(__file__), "README.md")
with open(readme_path, "r", encoding="utf-8") as f:
    long_description = f.read()

# Read requirements
requirements_path = os.path.join(os.path.dirname(__file__), "requirements.txt")
with open(requirements_path, "r", encoding="utf-8") as f:
    requirements = [line.strip() for line in f if line.strip() and not line.startswith("#")]

setup(
    name="unitree-gr00t-integration",
    version="1.0.0",
    description="Integration of Unitree G1 robot with GR00T-N1.5 for manipulation tasks",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Astro-Q",
    python_requires=">=3.10",
    install_requires=requirements,
    packages=find_packages(exclude=["tests", "docs"]),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
)
