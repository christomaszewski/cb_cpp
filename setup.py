import os
from setuptools import setup

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "cb_cpp",
    version = "0.0.1",
    author = "Christopher 'ckt' Tomaszewski",
    author_email = "christomaszewski@gmail.com",
    description = ("A reference implementation of the Constraint-Based Coverage Path Planning (CB-CPP) framework"),
    license = "BSD",
    keywords = "constraint-based constraint coverage planning robotics",
    url = "https://github.com/christomaszewski/cb_cpp.git",
    packages=['cb_cpp', 'tests', 'examples'],
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Utilities",
        "License :: OSI Approved :: BSD License",
    ],
)