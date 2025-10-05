# Scripts Directory

This directory contains utility scripts that are not part of the core ROS package.

## Contents

### VilotaSoftwareInstaller_v2.0-bba1a93f.py

Utility script for downloading and installing Vilota software binaries.

**Purpose**: Installs Vilota VK180 SDK dependencies including eCAL and Rerun.

**Usage**:
```bash
cd scripts
python3 VilotaSoftwareInstaller_v2.0-bba1a93f.py
```

**Requirements**:
- Python 3
- Internet connection
- Sudo access

**Supported Platforms**:
- Ubuntu 20.04 / 22.04 / 24.04 (x86_64, aarch64)
- Debian 12 (aarch64)

**Note**: This is a third-party utility script for installing vision system dependencies and is not required for basic PX4 navigation functionality.
