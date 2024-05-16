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

    def _is_ready_for_pnp(self, cargo, cur_task_id):
        cur_task = self.tasks_map[cur_task_id]
        if cargo['last_placed'] == "Never":
            return True
        ready_time = (cargo['last_placed']
                + rospy.Duration.from_sec(cur_task['duration']))
        if rospy.Time.now() > ready_time:
            return True
        else:
            return False

    def _next_station_id(self, next_task_id):
        stations = self.tasks_to_stations[next_task_id]
        for station in stations:
            if station['free']:
                return station['id']
        return None

    def _pnp_cargo(self, cargo, station_id):
        self._pick_and_place(cargo['id'], station_id)
        cargo['last_placed'] = rospy.Time.now()
        if station_id != 'drop_off_zone':
            cargo['cur_task_idx'] += 1
            self.station_map[station_id]['free'] = False
        if cargo['cur_task_idx'] != 0:
            self.station_map[cargo['last_station']]['free'] = True
        cargo['last_station'] = station_id

    def _pnp_dropoff(self, cargo):
        self._pnp_cargo(cargo, 'drop_off_zone')

    def _pick_and_place(self, cargo_id, station_id):
        self.arm_ctrl.pick_up(cargo_id)
        self.arm_ctrl.place_at(station_id) 

    def _last_task_not_done(self, cur_task_idx, tasks):
        return cur_task_idx < len(tasks) - 1

    def _not_done(self, cargo_id):
        return cargo_id not in self.done_cargoes

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            with self.seen_cargoes_lock:
                for cargo_id in self.seen_cargoes:
                    cargo = self.cargo_map[cargo_id]
                    cur_task_idx = cargo['cur_task_idx']
                    tasks = self.ctype_map[cargo['type']]['tasks']
                    cur_task_id = tasks[cur_task_idx]

                    if (self._not_done(cargo_id)
                            and self._is_ready_for_pnp(cargo, cur_task_id)):
                        if self._last_task_not_done(cur_task_idx, tasks):
                            next_task_id = tasks[cur_task_idx + 1]
                            station_id = self._next_station_id(next_task_id)
                            if station_id is not None:
                                self._pnp_cargo(cargo, station_id)
                        else: 
                            self._pnp_dropoff(cargo)
                            self.done_cargoes.add(cargo_id)

if __name__ == '__main__':
    rospy.init_node('pnp_controller')
    PnpController().run()

