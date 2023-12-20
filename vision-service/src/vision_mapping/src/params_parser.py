import yaml
from enum import Enum
from typing import List

class NodeOutput(Enum):
    Color = "color"

class NodeProfile:
    def __init__(self):
        self.name = ""
        self.hue_up = 0.0
        self.sat_up = 0.0
        self.val_up = 0.0
        self.hue_low = 0.0
        self.sat_low = 0.0
        self.val_low = 0.0

class NodeConfig:
    def __init__(self):
        self.debug = False
        self.output = NodeOutput.Color
        self.minimum = 0.0
        self.profiles = []

def node_output_to_string(output):
    return "Color"

def node_profile_to_string(profile):
    return (
        f"Name: {profile.name}, Hue Up: {profile.hue_up}, Sat Up: {profile.sat_up}, "
        f"Val Up: {profile.val_up}, Hue Low: {profile.hue_low}, Sat Low: {profile.sat_low}, "
        f"Val Low: {profile.val_low}"
    )

class ParamParser:
    def __init__(self, path):
        self.path = path

    def parse(self):
        config = NodeConfig()

        try:
            with open(self.path, "r") as file:
                yaml_data = yaml.safe_load(file)

                if yaml_data is not None and "detection" in yaml_data:
                    config_data = yaml_data["detection"]

                    config.debug = config_data.get("debug", False)

                    output = config_data.get("output", "")
                    config.output = NodeOutput(output)

                    config.minimum = config_data.get("minimum", 0.0)
                    config.grid_width = config_data.get("grid_width", 10)
                    config.grid_height = config_data.get("grid_height", 10)
                    config.camera_view = config_data.get("camera_view", False)
                    config.birdseye_view = config_data.get("birdseye_view", False)
                    config.mask_view = config_data.get("mask_view", False)
                    config.grid_view = config_data.get("grid_view", False)

                    profiles_data = config_data.get("profiles", [])
                    print("Profiles_data", profiles_data)
                    for profile_data in profiles_data:
                        print("Profile data:", profile_data)
                        profile = NodeProfile()
                        
                        if isinstance(profile_data, dict):
                            profile.name = profile_data.get("name")
                            profile.hue_up = profile_data.get("hue_up", 0.0)
                            profile.sat_up = profile_data.get("sat_up", 0.0)
                            profile.val_up = profile_data.get("val_up", 0.0)
                            profile.hue_low = profile_data.get("hue_low", 0.0)
                            profile.sat_low = profile_data.get("sat_low", 0.0)
                            profile.val_low = profile_data.get("val_low", 0.0)

                            config.profiles.append(profile)
                        else:
                            print(f"Invalid profile_data format in {self.path}: {profile_data}")

        except FileNotFoundError:
            print(f"File not found: {self.path}")

        return config

    def to_string(self, config):
        profiles_string = "\n".join(["  " + node_profile_to_string(profile) for profile in config.profiles])

        return (
            f"\nDebug: {config.debug}, Output: {node_output_to_string(config.output)}, "
            f"Minimum: {config.minimum}\nProfiles:\n{profiles_string}"
        )
