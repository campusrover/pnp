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
        self.station_map = self._get_map(configs, 'stations')
        self.tasks_map = self._get_map(configs, 'tasks')
        self.ctype_map = self._get_map(configs, 'cargoTypes')
        self.cargo_map = self._get_map(configs, 'cargoes')
        self.tasks_to_stations = self._get_tasks_to_stations_map(configs)
        self.seen_cargoes_sub = rospy.Subscriber('seen_cargoes',
                String,
                self._seen_cargoes_handler)
        self.seen_cargoes = set()
        self.seen_cargoes_lock = Lock()
        self.done_cargoes = set()

    def _seen_cargoes_handler(self, data):
        with self.seen_cargoes_lock:
            self.seen_cargoes.add(data.data)

    def _get_configs(self):
        abs_path_to_cur_file = os.path.dirname(__file__)
        config_file_path = os.path.join(abs_path_to_cur_file,
                '../config/config.json')
        with open(config_file_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _get_map(self, configs, key):
        res = {}
        for obj in configs[key]:
            res[obj['id']] = obj
        return res

    def _get_tasks_to_stations_map(self, configs):
        res = {}
        for task in configs['tasks']:
            res[task['id']] = [] 
        for station in configs['stations']:
            res[station['task_id']].append(station)
        return res

    def _is_ready(self, cargo, cur_task):
        if cargo['last_placed'] == "Never":
            return true
        ready_time = cargo['last_placed'] + cur_task['duration']
        if rospy.Time.now() > ready_time:
            return true
        else:
            return false

    def _next_station_id(self, next_task):
        stations = self.tasks_to_stations[next_task]
        for station in stations:
            if station['free']:
                return station['id']
        return None

    def _pnp_cargo(self, cargo_id, station_id):
        cargo['cur_task_idx'] += 1
        self._pick_and_place(cargo_id, station_id)
        cargo['last_placed'] = rospy.Time.now()
        cargo['last_station'] = station_id
        self.station_map[station_id]['free'] = false
        if cargo['cur_task_idx'] != 0:
            self.station_map[cargo['last_station']]['free'] = true

    def _pnp_dropoff(self, cargo_id):
        self._pick_and_place(cargo_id, 'drop_off_zone')

    def _pick_and_place(self, cargo_id, station_id):
        self.arm_ctrl.pick_up(cargo_id)
        self.arm_ctrl.place_at(station_id) 

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            with self.seen_cargoes_lock:
                for cargo_id in self.seen_cargoes:
                    cargo = self.cargo_map[cargo_id]
                    cur_task_idx = cargo['cur_task_idx']
                    tasks = self.ctype_map[cargo['type']]['tasks']
                    cur_task = tasks[cur_task_idx]

                    if (cargo['id'] not in self.done_cargoes and 
                            self._is_ready(cargo, cur_task)):
                        if cur_task_idx < len(tasks) - 1:
                            next_task = tasks[cur_task_idx + 1]
                            station_id = self._next_station_id(next_task)
                            if station_id is not None:
                                self._pnp_cargo(cargo_id, station_id)
                        else: 
                            self._pnp_dropoff(cargo_id)
                            self.done_cargoes.add(cargo_id)

if __name__ == '__main__':
    rospy.init_node('pnp_controller')
    PnpController().run()

