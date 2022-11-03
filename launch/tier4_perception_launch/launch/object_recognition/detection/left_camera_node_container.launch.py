# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch import LaunchContext

import yaml




def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    context = LaunchContext()

    use_multithread = "True"
    use_intra_process = "True"

    #tensorrt yolo params
    input_image = "/lucid_vision/camera_2/image"
    input_camera_info= "/lucid_vision/camera_2/camera_info"

    yolo_type = "yolov3"
    label_file = "coco.names"

    output_topic= "rois1"
    engine_file = FindPackageShare("tensorrt_yolo").perform(context) + "/data/"+ yolo_type + ".engine"
    calib_image_directory= FindPackageShare("tensorrt_yolo").perform(context) + "/calib_image/"

    mode = "FP32"
    gpu_id = 1

    tensorrt_config_path = FindPackageShare('tensorrt_yolo').perform(context)+ "/config/" + yolo_type + ".param.yaml"

    with open(tensorrt_config_path, "r") as f:
        tensorrt_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]



    camera_param_path = "/home/volt/projects/volt_drivers_ws/src/arena_camera/param/left_camera.param.yaml"
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result


    container = ComposableNodeContainer(
        name="left_camera_node_container",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            # ComposableNode(
            #     package="image_transport_decompressor",
            #     plugin="image_preprocessor::ImageTransportDecompressor",
            #     name="front_camera_image_decompressor",
            #     parameters=[{"encoding": "rgb8"}],
            #     remappings=[
            #         (
            #             "~/input/compressed_image", input_image + "/compressed",
            #         ),
            #         ("~/output/raw_image", input_image),
            #     ],
            #     extra_arguments=[
            #         {"use_intra_process_comms": bool(use_intra_process)}
            #     ],
            # ),
            ComposableNode(
                package='image_rectifier',
                plugin='image_preprocessor::ImageRectifier',
                name='rectify_left_camera_image_node',
                # Remap subscribers and publishers
                remappings=[
                    ('image', input_image),
                    ('camera_info', input_camera_info),
                    ('image_rect', 'image_rect_left')
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
            ComposableNode(
                package="tensorrt_yolo",
                plugin="object_recognition::TensorrtYoloNodelet",
                name="left_camera_tensorrt_yolo",
                parameters=[
                    {
                        "onnx_file": FindPackageShare("tensorrt_yolo").perform(context) +  "/data/" + yolo_type + ".onnx",
                        "engine_file": engine_file,
                        "label_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + label_file,
                        "calib_image_directory": calib_image_directory,
                        "calib_cache_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + yolo_type + ".cache",
                        "mode": mode,
                        "gpu_id": gpu_id,
                        "num_anchors": tensorrt_yaml_param['num_anchors'],
                        "anchors": tensorrt_yaml_param['anchors'],
                        "scale_x_y": tensorrt_yaml_param['scale_x_y'],
                        "score_threshold": tensorrt_yaml_param['score_threshold'],
                        "iou_thresh": tensorrt_yaml_param['iou_thresh'],
                        "detections_per_im": tensorrt_yaml_param['detections_per_im'],
                        "use_darknet_layer": tensorrt_yaml_param['use_darknet_layer'],
                        "ignore_thresh": tensorrt_yaml_param['ignore_thresh'],
                    }
                ],
                remappings=[
                    ("in/image", 'image_rect_left'),
                    ("out/objects", output_topic),
                    ("out/image", output_topic + "/debug/image"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
            ComposableNode(
                package="arena_camera",
                plugin="ArenaCameraNode",
                name="left_arena_camera_node",
                parameters=[{
                              "camera_name": camera_yaml_param['camera_name'],
                              "frame_id": camera_yaml_param['frame_id'],
                              "pixel_format": camera_yaml_param['pixel_format'],
                              "serial_no": camera_yaml_param['serial_no'],
                              "fps": camera_yaml_param['fps'],
                              "horizontal_binning": camera_yaml_param['horizontal_binning'],
                              "vertical_binning": camera_yaml_param['vertical_binning'],
                              "resize_image": camera_yaml_param['resize_image'],
                              "camera_info_url": camera_yaml_param['camera_info_url'],
                              "exposure_auto": camera_yaml_param['exposure_auto'],
                              "exposure_target": camera_yaml_param['exposure_target'],
                              "gain_auto": camera_yaml_param['gain_auto'],
                              "gain_target": camera_yaml_param['gain_target'],
                              "gamma_target": camera_yaml_param['gamma_target'],
                             }],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": bool(use_intra_process)}
                ],
            ),
        ],
        output="both",
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(use_multithread),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(use_multithread),
    )

    return LaunchDescription(
        [
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            container,
        ]
    )
