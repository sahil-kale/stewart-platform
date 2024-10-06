import subprocess
import os
import argparse


def main(deploy: bool = False):
    # chdir to "arduino"
    os.chdir("arduino")

    # Run the build script with platformio
    subprocess.run("pio run -e megaatmega2560", shell=True, check=True)
    # If deploy is True, run the upload script
    if deploy:
        subprocess.run(
            "pio run --target upload -e megaatmega2560", shell=True, check=True
        )


if __name__ == "__main__":
    # Argument parser to handle deploy option
    parser = argparse.ArgumentParser(
        description="Build and optionally deploy the project."
    )
    parser.add_argument(
        "--deploy",
        action="store_true",
        help="Deploy the project to the connected device",
    )
    args = parser.parse_args()

    main(args.deploy)

# if monitoring is ever needed run pio device monitor
