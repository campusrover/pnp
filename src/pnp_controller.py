#!/usr/bin/env python3

import os
import json
import rospy
import tf2_ros
from std_msgs.msg import String
from arm_controller import ArmController
from threading import Lock

class PnpController:

    def __init__(self):
        self.arm_ctrl = ArmController()
        configs = self._get_configs() 
        self.stations = configs['stations']
        self.tasks = configs['tasks']
        self.cargo_types = configs['cargoTypes']
        self.cargoes = configs['cargoes']
        self.seen_cargoes_sub = rospy.Subscriber('seen_cargoes',
                String,
                self._seen_cargoes_handler)
        self.seen_cargoes = set()
        self.seen_cargoes_lock = Lock()

    def _seen_cargoes_handler(self, data):
        with self.seen_cargoes_lock:
            self.seen_cargoes.add(data.data)
            rospy.loginfo(f'Detected {data.data}')

    def _get_configs(self):
        abs_path_to_cur_file = os.path.dirname(__file__)
        config_file_path = os.path.join(abs_path_to_cur_file,
                '../config/config.json')
        with open(config_file_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _pick_and_place(self, cf_id, ws_frame_name):
        self.arm_ctrl.pick_up(cf_id)
        self.arm_ctrl.place_at(ws_frame_name)

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            with self.seen_cargoes_lock:
                for cargo_id in self.seen_cargoes:
                    

if __name__ == '__main__':
    rospy.init_node('asm_line_controller')
    PnpController().run()

