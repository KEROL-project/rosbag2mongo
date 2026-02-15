#!/usr/bin/env python3
from typing import Any, Sequence, Dict
from importlib import import_module
import os
import threading
from collections import deque
from pathlib import Path
import pymongo as pm

import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader as RosbagReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from rosbag2mongo.messages import msg_utility_packages

class Rosbag2MongoConverter(Node):
    def __init__(self, name='rosbag2mongo_converter'):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.bag_file_path = self.get_parameter_or('bag_file_path',
                                                   rclpy.Parameter('bag_file_path', rclpy.Parameter.Type.STRING, '')).value
        self.lowest_frequency_topic = self.get_parameter_or('lowest_frequency_topic',
                                                            rclpy.Parameter('lowest_frequency_topic', rclpy.Parameter.Type.STRING, '')).value
        self.observation_topics = self.get_parameter_or('observation_topics',
                                                        rclpy.Parameter('observation_topics', rclpy.Parameter.Type.STRING_ARRAY, [])).value
        self.observation_param_names = self.get_parameter_or('observation_param_names',
                                                             rclpy.Parameter('observation_param_names', rclpy.Parameter.Type.STRING_ARRAY, [])).value
        self.action_topic = self.get_parameter_or('action_topic',
                                                  rclpy.Parameter('action_topic', rclpy.Parameter.Type.STRING, '')).value
        self.bag_file_path = self.get_parameter_or('bag_file_path',
                                                   rclpy.Parameter('bag_file_path', rclpy.Parameter.Type.STRING, '')).value
        self.dataset_name = self.get_parameter_or('dataset_name',
                                                  rclpy.Parameter('dataset_name', rclpy.Parameter.Type.STRING, '')).value
        self.robot_name = self.get_parameter_or('robot_name',
                                                 rclpy.Parameter('robot_name', rclpy.Parameter.Type.STRING, '')).value
        self.msg_package_names = self.get_parameter_or('msg_package_names',
                                                       rclpy.Parameter('msg_package_names', rclpy.Parameter.Type.STRING_ARRAY, [])).value
        self.msg_package_paths = self.get_parameter_or('msg_package_paths',
                                                       rclpy.Parameter('msg_package_paths', rclpy.Parameter.Type.STRING_ARRAY, [])).value


        self.observation_topics_to_names = {}
        for i in range(len(self.observation_topics)):
            self.observation_topics_to_names[self.observation_topics[i]] = self.observation_param_names[i]

        self.get_logger().info(str(self.msg_package_names))
        self.get_logger().info(str(self.msg_package_paths))
        self.get_logger().info(self.lowest_frequency_topic)
        self.get_logger().info(str(self.observation_topics))
        self.get_logger().info(str(self.observation_topics_to_names))
        self.get_logger().info(self.action_topic)
        self.get_logger().info(self.bag_file_path)
        self.get_logger().info(self.dataset_name)
        self.get_logger().info(self.robot_name)
        self.typestore = get_typestore(Stores.LATEST)

    def register_custom_types(self, package_names: Sequence[str], package_paths: Sequence[str]):
        """Registers custom message types in the rosbags typestore.
        The messages are expected to be located in {package_path}/msg,
        and the type will be registered under the key {package_name}/msg/MessageName.

        Keyword arguments:
        package_names: Sequence[str] -- Names of the package containing custom messages
                                        (required so that the type name can be specified)
        package_paths: Sequence[str] -- Absolute paths of the packages containing custom messages
                                        (required so that the message definitions can be read)

        """
        try:
            for (package_name, package_path) in zip(package_names, package_paths):
                msg_path = os.path.join(package_path, 'msg')
                new_types = {}
                for msg_file in os.listdir(msg_path):
                    extension = msg_file.split('.')[1]
                    if extension != 'msg':
                        continue

                    msg_file_path = os.path.join(msg_path, msg_file)
                    msg_text = Path(msg_file_path).read_text()

                    # ignoring the .msg part of the file name
                    msg_name = msg_file.split('.')[0]

                    # setting the type name as {package_name}/msg/MessageName
                    msg_type_name = os.path.join(package_name, 'msg', msg_name)

                    self.get_logger().info(f'Found message of type {msg_type_name}')
                    new_types.update(get_types_from_msg(msg_text, msg_type_name))

            self.get_logger().info('Registering new messages')
            self.typestore.register(new_types)
            self.get_logger().info('New messages registered')
        except Exception as exc:
            self.get_logger().error("Exception: %s" % str(exc))

    def convert_bag_to_mongo(self, bag_path: str,
                             observation_topics: Sequence[str],
                             lowest_frequency_topic: str,
                             observation_topic_to_names: Dict[str,str],
                             action_topic: str,
                             dataset_name: str,
                             robot_name: str):
        """Converts a collection of rosbags to a MongoDB dataset; during the conversion, the observation
        topics are synchronised with respect to the lowest-frequency topic.

        Keyword arguments:
        bag_path: str -- Absolute path of a directory containing rosbags
        observation_topics: Sequence[str]: Names of the topics that should be considered as observations
        lowest_frequency_topic: str -- Name of an observation topic that is published at the lowest frequency;
                                       the other observation topics are then synchronised with respect to it
        observation_topic_to_names: Dict[str,str] -- A dictionary in which the keys are observation topic names (from observation_topics)
                                                     and the values are observation names in the Mongo dataset
                                                     (e.g. {'/camera/image/raw': 'image'})
        action_topic: str -- Topic that is used for extracting action data
        dataset_name: str --- Name of the output dataset
        robot_name: str --- Name of the robot that produced the dataset

        """
        # we keep some history of topic-specific messages so that we can synchronise the data
        observation_topics_without_lower_frequency_topic = [x for x in observation_topics if x != lowest_frequency_topic]

        ActionConverterClass = getattr(import_module(f'rosbag2mongo.action_converters.{robot_name}'), 'ActionConverter')
        action_converter = ActionConverterClass()

        db_client = Rosbag2MongoConverter.get_db_client()
        database = db_client[robot_name]

        episode_step = {'is_first': False, 'is_last': False,
                        'observation': {name:None for name in observation_topic_to_names.values()},
                        'action': None}
        try:
            topics_to_monitor = set(observation_topics).union([action_topic])
            for episode_id, bag_dir_name in enumerate(os.listdir(bag_path)):
                topic_data = {topic: deque(maxlen=10) for topic in topics_to_monitor}
                bag_dir_path = os.path.join(bag_path, bag_dir_name)
                self.get_logger().info('Processing rosbag %s' % bag_dir_path)

                with RosbagReader(bag_dir_path) as reader:
                    step_id = 0
                    for connection, timestamp, raw_data in reader.messages():
                        if connection.topic not in topics_to_monitor:
                            continue

                        msg = self.typestore.deserialize_cdr(raw_data, connection.msgtype)
                        topic_data[connection.topic].append((msg, connection.msgtype))
                        if connection.topic == lowest_frequency_topic:
                            # we ignore the message in case some of the other observation
                            # messages or the action message haven't been received yet
                            ignore_msg = False
                            for topic_name in topics_to_monitor:
                                if not topic_data[topic_name]:
                                    ignore_msg = True

                            # we also ignore the message if we still don't have at least two
                            # action messages for extracting the applied action
                            if len(topic_data[action_topic]) < 2:
                                ignore_msg = True

                            if ignore_msg:
                                self.get_logger().info('Ignoring message of type %s because not all topics have been synchronised yet' % connection.topic)
                                continue

                            episode_step['observation'][observation_topic_to_names[lowest_frequency_topic]] = self.extract_msg_data(msg, connection.msgtype)
                            for topic in observation_topics_without_lower_frequency_topic:
                                (sync_msg, sync_msg_type) = topic_data[topic][-1]
                                episode_step['observation'][observation_topic_to_names[topic]] = self.extract_msg_data(sync_msg, sync_msg_type)

                            # we extract an action from the last two messages on the action topic (e.g. as an end effector displacement)
                            episode_step['action'] = action_converter.get_actions(prev_msg=topic_data[action_topic][-2][0],
                                                                                  last_msg=topic_data[action_topic][-1][0])
                            episode_step['step_id'] = step_id
                            episode_step['episode_id'] = episode_id

                            self.get_logger().info("Saving episode step %d to database %s" %(step_id, robot_name))
                            collection = database[f'{dataset_name}-ep{episode_id}']
                            collection.insert_one(episode_step)

                            step_id += 1
                            episode_step = {'is_first': False, 'is_last': False,
                                            'observation': {name:None for name in observation_topic_to_names.values()},
                                            'action': None}
        except Exception as exc:
            self.get_logger().error("Exception: %s" % str(exc))
            db_client.close()

    def extract_msg_data(self, msg: Any, msg_type: str) -> Any:
        """Extracts data from 'msg' of type 'msg_type'.

        Keyword arguments:
        msg: Any -- ROS message from which data should be extracted
        msg_type: str -- ROS message type

        """
        try:
            utility_package, utility_class_name = msg_utility_packages[msg_type]
            MsgUtilityClass = getattr(import_module(utility_package), utility_class_name)
            utility_obj = MsgUtilityClass(msg)
            return utility_obj.get_data()
        except Exception as exc:
            self.get_logger().error("Exception: %s" % str(exc))

    @staticmethod
    def get_db_client():
        """Returns a MongoDB client at <host>:<port>. By default,
        <host> is "localhost" and <port> is 27017, but these values can
        be overriden by setting the environment variables "DB_HOST" and
        "DB_PORT" respectively.

        Method taken from https://github.com/ropod-project/black-box-tools/blob/master/black_box_tools/db_utils.py
        """
        (host, port) = Rosbag2MongoConverter.get_db_host_and_port()
        client = pm.MongoClient(host=host, port=port)
        return client

    @staticmethod
    def get_db_host_and_port():
        """Returns a (host, port) tuple which is ("localhost", 27017) by default,
        but the values can be overridden by setting the environment variables
        "DB_HOST" and "DB_PORT" respectively.

        Method taken from https://github.com/ropod-project/black-box-tools/blob/master/black_box_tools/db_utils.py
        """
        host = 'localhost'
        port = 27017
        if 'DB_HOST' in os.environ:
            host = os.environ['DB_HOST']
        if 'DB_PORT' in os.environ:
            port = int(os.environ['DB_PORT'])
        return (host, port)

def main(args=None):
    rclpy.init(args=args)
    rosbag2mongo_converter = Rosbag2MongoConverter()

    rate = rosbag2mongo_converter.create_rate(5, rosbag2mongo_converter.get_clock())
    spin_thread = threading.Thread(target=rclpy.spin, args=(rosbag2mongo_converter,), daemon=True)
    spin_thread.start()

    try:
        rosbag2mongo_converter.register_custom_types(package_names=rosbag2mongo_converter.msg_package_names,
                                                    package_paths=rosbag2mongo_converter.msg_package_paths)

        rosbag2mongo_converter.convert_bag_to_mongo(bag_path=rosbag2mongo_converter.bag_file_path,
                                                    observation_topics=rosbag2mongo_converter.observation_topics,
                                                    observation_topic_to_names=rosbag2mongo_converter.observation_topics_to_names,
                                                    lowest_frequency_topic=rosbag2mongo_converter.lowest_frequency_topic,
                                                    action_topic=rosbag2mongo_converter.action_topic,
                                                    dataset_name=rosbag2mongo_converter.dataset_name,
                                                    robot_name=rosbag2mongo_converter.robot_name)
        while rclpy.ok():
            rate.sleep()
    except Exception as exc:
        rosbag2mongo_converter.get_logger().error("Exception: %s" % str(exc))

    print('Destroying data converter node')
    rosbag2mongo_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
