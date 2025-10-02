#!/usr/bin/env python3
"""This is a script to download and install Vilota Binaries"""

import argparse
import getpass
import http.client
import json
import os
import platform
import subprocess
import sys
import requests
import re

# Constants for colored output
GREEN = "\033[92m"
RED = "\033[91m"
BLUE = "\033[94m"
YELLOW = "\033[93m"
RESET = "\033[0m"

VILOTA_API_URL = "files.vilota.ai"
VILOTA_PATH = "/var/lib/vilota"
SCRIPT_PATH_DEP = "/scripts/"
SCRIPT_PATH_CICD = "/released/scripts/"
SUPPORTED_PLATFORMS = {
    ("x86_64", "Linux", "20.04"): "Ubuntu20.04_x86_64",
    ("x86_64", "Linux", "22.04"): "Ubuntu22.04_x86_64",
    ("aarch64", "Linux", "20.04"): "Ubuntu20.04_aarch64",
    ("aarch64", "Linux", "22.04"): "Ubuntu22.04_aarch64",
    ("aarch64", "Linux", "12"): "Debian12_aarch64",
    ("x86_64", "Linux", "24.04"): "Ubuntu24.04_x86_64",
    ("aarch64", "Linux", "24.04"): "Ubuntu24.04_aarch64",
}

ECAL_VERSION = "5.11"
RERUN_VERSION = "0.20.1"


def print_colored(message, color):
    print(f"{color}{message}{RESET}")


def print_success(message):
    print_colored(message, GREEN)


def print_error(message):
    print_colored(message, RED)


def print_info(message):
    message = "[Info] " + message
    print_colored(message, BLUE)


def print_warning(message):
    print_colored(message, YELLOW)

def ping(host):
    """Ping a host and return True if reachable, False otherwise."""
    try:
        # "-c 1" means send only 1 packet; this works for Linux/macOS. Use "-n 1" for Windows.
        subprocess.check_output(["sudo", "ping", "-c", "1", host], stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError:
        return False
    
def check_connectivity():
    if ping("1.1.1.1"):
        print("Ping to 1.1.1.1 successful.")
        if ping("files.vilota.ai"):
            print("Ping to files.vilota.ai successful. No issues detected.")
        else:
            raise Exception("Ping to files.vilota.ai failed. DNS resolution issue suspected.")
    else:
        raise Exception("Ping to 1.1.1.1 failed. No network connectivity.")

def run_command(command):
    """
    Example usage:
    command = ['ls', '-l']
    output, error, exit_status = run_command(command)
    """
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        output = result.stdout
        error = result.stderr
        exit_status = result.returncode
    except subprocess.CalledProcessError as e:
        output = e.stdout
        error = e.stderr
        exit_status = e.returncode
    except Exception as e:
        output = ""
        error = str(e)
        exit_status = -1

    return output, error, exit_status


def determine_platform_suffix():
    os_version = platform.system()
    os_release = os.popen("lsb_release -rs").read().strip()
    cpu_arch = platform.machine()

    platform_key = (cpu_arch, os_version, os_release)
    platform_suffix = SUPPORTED_PLATFORMS.get(platform_key)

    if platform_suffix:
        print_info(f"System info: {platform_suffix.replace('_', ' ')}")
        return platform_suffix

    print_warning(f"OS: {os_version} {os_release}")
    print_warning(f"Arch: {cpu_arch}")
    print_error("[Error] Unsupported platform or OS.")
    sys.exit(1)


def determine_package_name(package, platform_suffix):
    return f"{package}_{platform_suffix}.deb"


def determine_package_path(directory, package_name):
    return f"/{directory}/{package_name}"


def api_request(conn, method, endpoint, payload, headers):
    conn.request(method, endpoint, json.dumps(payload), headers)
    response = conn.getresponse()
    data = response.read().decode("utf-8")
    return response.status, json.loads(data) if data else {}


def login(username, password):
    conn = http.client.HTTPSConnection("files.vilota.ai")
    payload = json.dumps({"username": username, "password": password})
    headers = {"Content-Type": "application/json"}
    conn.request("POST", "/api/auth/login", payload, headers)
    res = conn.getresponse()
    response_data = res.read().decode("utf-8")

    if res.status == 200:
        try:
            response_json = json.loads(response_data)
            data = response_json.get("data")
            if data and "token" in data:
                print_success("Login Successful!")
                return True, data["token"]
            else:
                print_error(
                    "Token not received.\n"
                    "This may be due to an incorrect username or password.\n"
                    "Please get in touch with a Vilota Engineer for assistance.\n"
                )
                return False, None
        except json.JSONDecodeError:
            print_error("Failed to decode JSON response.\n")
            return False, None
    else:
        print_error("Login failed\n")
        return False, None


def get_file_info(path, token, endpoint):
    conn = http.client.HTTPSConnection(VILOTA_API_URL)
    payload = {"path": path, "password": "", "page": 1, "per_page": 0, "refresh": True} # force refresh
    headers = {"Authorization": token, "Content-Type": "application/json"}

    try:
        status, data = api_request(conn, "POST", endpoint, payload, headers)
        if data is None:
            raise ValueError("No data received from API")
        if data.get("data") is None:
            raise ValueError(f"{data.get('message', 'Unknown error')}")

        if endpoint == "/api/fs/list":
            content = data.get("data", {}).get("content", [])
            if not content:
                raise ValueError("No Script Found in server")
            return content[0].get("name")
        elif endpoint == "/api/fs/get":
            raw_url = data.get("data", {}).get("raw_url")
            if not raw_url:
                raise ValueError("No raw URL found")
            return raw_url
        else:
            raise RuntimeError(f"endpoint not supported: {endpoint}")
    
    except (KeyError, IndexError, TypeError, ValueError) as e:
        print(f"Error retrieving file information: {e}")
        sys.exit(1)


def download_file(url, filename, path = "./"):
    response = requests.get(url, stream=True, timeout=60) # allow 60 seconds to download
    total_size = int(response.headers.get('content-length', 0))  # Total size in bytes
    total_size_mb = total_size / (1024 * 1024)
    downloaded_size = 0  # Track the number of bytes downloaded

    # Define a simple function to print a progress bar
    def print_progress_bar(percentage, total, width=40):
        completed = int(percentage * width)
        bar = f"[{'#' * completed}{'.' * (width - completed)}] {percentage * 100:.2f}% ({total:.2f} MB)"
        sys.stdout.write('\r' + bar)
        sys.stdout.flush()

    file_location = os.path.join(path, filename)
    # print_info(f"downloading {url} as {file_location}")
    with open(file_location, "wb") as file:
        for chunk in response.iter_content(chunk_size=2048):
            if chunk:
                file.write(chunk)
                downloaded_size += len(chunk)
                
                # Calculate the percentage of completion and print the progress bar
                progress_percentage = downloaded_size / total_size
                print_progress_bar(progress_percentage, total_size_mb)

    print("") # extra line
    print(f"File downloaded successfully to {file_location}")


def install_deb(filename, path = "./"):
    subprocess.run(
        ["sudo", "apt", "install", "-y", "--allow-downgrades", f"{path}/{filename}"],
        check=True,
    )


def describe_installation(filename):
    subprocess.run(["sudo", "apt", "policy", filename], check=True)


def add_to_path_and_bashrc():
    path_dir = "/opt/vilota/bin"
    bashrc_path = os.path.expanduser('~/.bashrc')

    # Read the current content of .bashrc
    with open(bashrc_path, 'r') as file:
        lines = file.readlines()

    # Check if the path_to_add is already in .bashrc
    path_export_line = f'export PATH=\"{path_dir}:$PATH\"\n'
    if path_export_line in lines:
        print(f"The path {path_dir} is already present in {bashrc_path}.")
        return
    
    # Append the export command to .bashrc
    with open(bashrc_path, 'a') as file:
        file.write(f'\n# Added by script\n{path_export_line}')
    
    print_success(f"Added {path_dir} to {bashrc_path}.")


def chown_vilota_dir():
    uid = os.getuid()
    command = f"sudo mkdir -p {VILOTA_PATH}"
    subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
    command = f"sudo chown {uid}:{uid} -R {VILOTA_PATH}"
    subprocess.run(command, shell=True, check=True, text=True, capture_output=True)


def is_ecal_installed(version_required):
    try:
        result = subprocess.run(
            ["apt", "policy", "ecal"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
        output = result.stdout.decode() + result.stderr.decode()
        if "Unable to locate package ecal" in output:
            return False
        
        # Extract the installed version from the output
        version = None
        for line in output.split('\n'):
            if 'Installed' in line:
                version = line.split(':')[1].strip()
                break
            
        # case where parsing failed
        if version is None:
            print_warning("[WARN] eCAL version is not detected. (parsing failed)")
            return False
            
        if not version.startswith(version_required):
            print_warning(f"[WARN] Ecal version not correct [{version}], please uninstall other versions and keep only {version_required}.x")
            return False
        
        return True
    except subprocess.CalledProcessError:
        print_warning("[WARN] is_ecal_installed CalledProcessError")
        return False
    except FileNotFoundError:
        print_warning("[WARN] is_ecal_installed FileNotFoundError")
        return False


def install_ecal(token, platform_suffix):
    if is_ecal_installed(ECAL_VERSION):
        print_info(f"eCAL {ECAL_VERSION}.x is already installed.")
        return

    if "Debian" in platform_suffix:
        print_info("Downloading Debian12 Deb from remote...")
        script_url = get_file_info(
            f"/ecal/ecal_Debian/eCAL-5.11.8-Debian12-arm.deb", token, "/api/fs/get"
        )
        download_file(script_url, "eCAL-5.11.8-Debian12-arm.deb", "/tmp")
        print_info("Installing eCAL...")
        subprocess.run(["sudo", "apt-get", "install", "-y", "/tmp/eCAL-5.11.8-Debian12-arm.deb"], check=True)
    else:
        subprocess.run(
            ["sudo", "add-apt-repository", "-y", "ppa:ecal/ecal-" + ECAL_VERSION], check=True
        )
        subprocess.run(["sudo", "apt-get", "update"], check=True, stdout=subprocess.PIPE)
        print_info(f"Installing eCAL {ECAL_VERSION}...")
        subprocess.run(["sudo", "apt-get", "-y", "install", "ecal"], check=True)

def is_rerun_installed(required_version):
    try:
        result = subprocess.run(
            [sys.executable, "-m", "pip", "show", "rerun-sdk"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )
        output = result.stdout.decode() + result.stderr.decode()

        # Check if the package is installed
        if f"Name: rerun-sdk" not in output:
            return False

        # Check if the installed version matches the required version
        installed_version = None
        for line in output.splitlines():
            if line.startswith("Version:"):
                installed_version = line.split(":")[1].strip()
                break

        if installed_version == required_version:
            return True
        else:
            print(f"[WARN] rerun-sdk version not correct. Installed: {installed_version}, Required: {required_version}")
            return False

    except subprocess.CalledProcessError:
        return False
    except FileNotFoundError:
        return False


def install_rerun(token, platform_suffix):
    if is_rerun_installed(RERUN_VERSION):
        print_info(f"rerun {RERUN_VERSION} is already installed.")
        return
    
    if "Debian" in platform_suffix:
        print_info("skip installation of rerun-sdk on Debian platform (expect error: externally-managed-environment)")
    else:
        print(f"\nInstalling rerun {RERUN_VERSION}...")
        subprocess.run([
                    sys.executable, 
                    "-m", 
                    "pip", 
                    "install", 
                    "--user", 
                    "rerun-sdk==" + RERUN_VERSION
                ], check=True)
def install_dependencies(platform_suffix):
    deps = ["build-essential",
            "cmake",
            "gpg-agent",
            "libopencv-dev",
            "libfmt-dev",
            "libprotobuf-dev",
            "libtbb-dev",
            "libboost-serialization-dev"]
    print_info(f"Install apt dependencies: {deps}")
    subprocess.run(
        [
            "sudo",
            "apt",
            "install",
            "-y",
            *deps
        ],
        check=True,
    )
    if "Debian" in platform_suffix:
        print_warning("skip installation of python dependencies on Debian. including python3-ecal5 and pycapnp")
    else:
        subprocess.run(
            [
                "sudo",
                "apt",
                "install",
                "-y",
                "python3-ecal5",
            ],
            check=True,
        )

        PYCAPNP_VERSION="2.0.0"
        subprocess.run(
            [
                sys.executable, 
                "-m", 
                "pip",
                "install",
                "--user",
                f"pycapnp=={PYCAPNP_VERSION}"
            ],
            check=True,
        )


def configure_movidius_rules():
    rule_line = 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'
    rules_file = "/etc/udev/rules.d/80-movidius.rules"

    if os.path.exists(rules_file):
        with open(rules_file, 'r') as file:
            for line in file:
                if rule_line in line:
                    print_info("movidius udev rule already exists. No changes needed.")
                    return

    subprocess.run(
        f'echo \'{rule_line}\' | sudo tee {rules_file}',
        shell=True,
        check=True,
    )

    subprocess.run(
        'sudo udevadm control --reload-rules && sudo udevadm trigger',
        shell=True,
        check=True,
    )

    print_info("Configuration added and udev rules reloaded.")


def main():
    if os.geteuid() == 0:
        print("This script should not be run as root.")
        sys.exit(1)

    parser = argparse.ArgumentParser(
        description="Install Vilota VisualKit packages.",
        epilog="Copyright(c) 2024 Vilota Pte Ltd. All rights reserved.",
    )
    parser.add_argument(
        "-s",
        "--vksystem",
        dest="with_vksystem",
        action="store_true",
        required=True,
        help="Install the VisualKit System.",
    )
    parser.add_argument(
        "-m",
        "--vkmanager",
        dest="with_vkmanager",
        action="store_true",
        help="Install the optional VisualKit Manager.",
    )
    parser.add_argument("-u", "--username", dest="username", help="Username for login.")
    parser.add_argument("-p", "--password", dest="password", help="Password for login.")
    parser.add_argument("-n", "--no-update-script", dest="no_update_script", help="Do not check for script update", action='store_true', default=False)
    args = parser.parse_args()

    username = args.username or input("Enter username: ")
    password = args.password or getpass.getpass("Enter password: ")

    print_info("checking network connectivity. This script requires Internet access to work.");
    check_connectivity()

    success, token = login(username, password)
    if not success:
        sys.exit(1)

    print_info("Checking and updating script")
    if username == "cicd":
        script_path = SCRIPT_PATH_CICD
    else:
        script_path = SCRIPT_PATH_DEP
    
    sha256sum_result = subprocess.run(
            ["sha256sum", __file__],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
    )

    current_sha256_short = sha256sum_result.stdout.decode()[:8]
    print(f"current installer script [{__file__}] has sha256 {current_sha256_short}")
    
    if args.no_update_script:
        print_info("Skip checking script updates from remote, use as it is...")
    else:
        remote_script_name = get_file_info(script_path, token, "/api/fs/list")

        match = re.search(r'-(\w+)\.py$', remote_script_name)
        assert(match)
        extracted = match.group(1)
        remote_sha256_short = extracted
        print(f"remote installer script [{remote_script_name}] has sha256 {remote_sha256_short}")


        if current_sha256_short != remote_sha256_short:
            print_info(
                f"New File Found [{remote_script_name}]. Downloading the latest script into /tmp/ directory"
            )

            script_url = get_file_info(
                f"{script_path}{remote_script_name}", token, "/api/fs/get"
            )
            download_file(script_url, remote_script_name, "/tmp/")
            print_info(
                f"Downloaded to /tmp/{remote_script_name}, please run again using python3"
            )
            text = input(f"Press Enter directly to automatically run the new script at /tmp/{remote_script_name}, or press enter any other key to quit...")

            if text == "":
                params = ["-u", username, "-p", password]
                if args.with_vksystem:
                    params.append("-s")
                if args.with_vkmanager:
                    params.append("-m")
                os.execvp('python3', [sys.executable, f"/tmp/{remote_script_name}"] + params)
            else:
                sys.exit(1)
        else:
            print_info("Current Script is the Latest")

    platform_suffix = determine_platform_suffix()
    
    install_ecal(token, platform_suffix)
    install_rerun(token, platform_suffix)
    install_dependencies(platform_suffix)

    packages = []
    if args.with_vksystem:
        packages.append(
            (
                "vk-system",
                determine_package_path(
                    "vk-system", determine_package_name("vk-system", platform_suffix)
                ),
            )
        )
    if args.with_vkmanager:
        packages.append(
            (
                "vk-manager-server",
                determine_package_path(
                    "vk-manager",
                    determine_package_name("vk-manager-server", platform_suffix),
                ),
            )
        )

    
    for name, path in packages:
        package_name = determine_package_name(name, platform_suffix)
        print_info(f"Downloading Binaries... ({package_name})")
        url = get_file_info(path, token, "/api/fs/get")
        download_file(url, package_name, "/tmp/")

    print_info("Installing Binaries...")
    for name, _ in packages:
        install_deb(determine_package_name(name, platform_suffix), "/tmp/")

    print_info("Showing Installation Information...")
    for name, _ in packages:
        describe_installation(name)

    add_to_path_and_bashrc()
    chown_vilota_dir()
    configure_movidius_rules()
    
    print_success("Installation Completed Successfully.")


if __name__ == "__main__":
    main()