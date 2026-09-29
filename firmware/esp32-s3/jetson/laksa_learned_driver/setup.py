from glob import glob
from setuptools import setup

package_name = "laksa_learned_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["console_page.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models", glob("models/*.npz") + glob("models/*.json")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Project LAKSA",
    maintainer_email="project-laksa@invalid.local",
    description="Imitation-learned LiDAR driving policy for the LAKSA LiDAR Cruise slot.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "learned_driver_node = laksa_learned_driver.driver_node:main",
        "policy_probe = laksa_learned_driver.policy_probe:main",
        "laksa_operator = laksa_learned_driver.operator_cli:main",
        "zed_perception = laksa_learned_driver.zed_perception_node:main",
        "laksa_console = laksa_learned_driver.console_node:main",
        "race_manager = laksa_learned_driver.race_node:main",
    ]},
)
