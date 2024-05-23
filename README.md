# PNP

This project is called 'PNP' for 'pick and place'. It can simulate a factory
floor, where parts are _picked up_ and _placed down_ during various stages of a
manufacturing process.

---
## Table of Contents
- [Video](#video)
- [What](#what)
- [Why](#why)
- [How](#how)
- [Next Steps](#next-steps)
---

## Video
[Watch the project in action!](https://drive.google.com/file/d/1KwD7RT69WAX70gCqWBaPEyARUFQieyVz/view?usp=sharing)

## What

PNP can simulate any factory floor that can be described within the constraints
of its JSON config file (`/config/config.json`).

For instance, PNP ships with an example floor that has two tasks: washing that 
takes 3 seconds, and welding that takes 2 seconds. 

```json   
"tasks": [
    {
        "id": "wash",
        "duration": 3
    },
    {
        "id": "weld",
        "duration": 2
    }
]
```

Tasks are executed at stations. The same task can be executed at more than one
station. For example, we see below that washing happens at both `station_0` and
`station_2`. This means that if a part needs to be washed, but `station_0` is
occupied, it can be handled in `station_2`. 

```json
"stations": [
    {
        "id": "station_0",
        "fid_id": 1,
        "task_id": "wash",
        "free": true 
    },
    {
        "id": "station_1",
        "fid_id": 2,
        "task_id": "weld",
        "free": true 
    },
    {
        "id": "station_2",
        "fid_id": 3,
        "task_id": "wash",
        "free": true 
    }
]
```

Thus, the mapping from tasks to stations can be one-to-many. Stations are
represented by fiducial markers. We see above, for instance, that `station_2` is
paired with a fiducial of id 3. The physical fiducial is marked in red below.

<p align="center">
    <kbd>
        <img src="./images/fiducial-3.png" />
    </kbd>
</p>

A cargo represents the set of parts that are moved from one station to another
during a manufacturing process. For example, a set of `raw_parts` is represented
by `cargo_0` below:

```json
{
    "id": "cargo_0",
    "fid_id": 10,
    "type": "raw_parts",
    "cur_task_idx": -1,
    "last_station": "None",
    "last_placed": "Never"
}
```

And we see that a cargo of type `raw_parts` needs to be washed, welded, then
washed:

```json
"cargoTypes": [
    {
        "id": "raw_parts",
        "tasks": ["wash", "weld", "wash"]     
    },
    {
        "id": "washed_parts",
        "tasks": ["weld", "wash"]
    }
]
```

Physically, a cargo is represented in PNP by a 3D-printed cube fixed with a
fiducial.

To see how these tasks, stations, and cargoes come together in a simulated
factory, watch [this recording of a
demo](https://drive.google.com/file/d/1KwD7RT69WAX70gCqWBaPEyARUFQieyVz/view?usp=sharing).

## Why

This project was developed by the Brandeis Robotics Lab as an example project
for future students. The aim was to demo how one can use the lab's PX-100
robotic arm to implement an autonomous pick and place system.

## How

The core of the vision and pick-and-place systems used by PNP were developed in
an earlier project. [Its
README](https://github.com/campusrover/fiducial_pick_and_place/blob/main/README.md)
explains both.

What is new in PNP is:

1. the automation of the pick-and-place action; and
2. the scheduling algorithm that manages when and where to pick and place
   cargoes.

The first is achieved by communication between the `pnp_frame_broadcaster` node
and the `pnp_controller` node. Whenever the former detects a cargo via the
vision system, it publishes a message to the latter over the `seen_cargoes`
topic. Next, the `pnp_controller` node picks up the detected cargo so long as it
satisfies the constraints of its scheduling algorithm.

Also new in PNP is its use of the [Brandeis Robotics Lab's
API](https://github.com/campusrover/brl_pxh_api) for the PX-100 arm rather than
Interbotix's official API.

## Next Steps

PNP can be readily extended. For example, we might add a feature that allows us
to add a success rate to each task. Then, the challenge for PNP would be to
automatically account for cases where a task fails.
 
