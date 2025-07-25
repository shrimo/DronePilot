from setuptools import setup, find_packages

setup(
    name="DronePilot",
    version="0.1.0",
    description="High-level MAVLink API for drone control using pymavlink",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Your Name",
    author_email="your.email@example.com",
    url="https://github.com/yourusername/dronepilot",
    packages=find_packages(),
    install_requires=[
        "pymavlink>=2.4.30"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
