#!/bin/bash
rostopic pub /ball/reaching_goal/goal exp_assignment2/PlanningActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    pose:
      position:
        x: 0.0
        y: 0.0
        z: -0.5
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0" 

