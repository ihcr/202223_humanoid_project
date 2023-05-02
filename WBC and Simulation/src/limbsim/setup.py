#!/usr/bin/env python

from os import path, walk
from setuptools import setup, find_packages

package_name = "limbsim"
package_version = "1.0.0"


def find_resources(package_name):
    """ Find the relative path of files under the resource folder. """
    resources = []
    package_dir = path.join("python", package_name)
    resources_dir = path.join(package_dir, "resources")

    for (root, _, files) in walk(resources_dir):
        for afile in files:
            if (
                afile != package_name
                and not afile.endswith(".DS_Store")
                and not afile.endswith(".py")
            ):
                rel_dir = path.relpath(root, package_dir)
                src = path.join(rel_dir, afile)
                resources.append(src)
    return resources


with open(path.join(path.dirname(path.realpath(__file__)), "readme.md"), "r") as fh:
    long_description = fh.read()

# Find the resource files.
resources = find_resources(package_name)

# Setup the package
setup(
    name=package_name,
    version=package_version,
    package_dir={'': 'python',},
    packages=find_packages(where='python'),
    package_data={package_name: resources},
    # required dependencies.
    install_requires=["setuptools", "pybullet", "importlib_resources", "commutils"],
    # optional (hack) dependencies.
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Jun Li",
    maintainer_email="junlileeds@gmail.com",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ihcr/limbsim",
    description="PyBullet wrapper for robots.",
    license="BSD-3-clause",
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)