# MoveIt! Boilerplate
By Dave Coleman

Quickly get started with MoveIt! in C++ with several easy to use classes described in the following subsections:

SEE IMAGE ``/docs/overview.png``
<img align="right" src="https://raw.github.com/davetcoleman/moveit_boilerplate/master/docs/demo.png" />

## Boilerplate

Inherit your MoveIt! from this base class to quickly give you:

 - Planning Scene Monitor
 - Robot Model
 - Current Robot State
 - Execution interface to controllers
 - Planning interface to OMPL, etc
 - Remote control interface to joysticks, interactive markers, etc
 - File read/write for joint and cartesian trajectory
 - Visual tools for debugging in Rviz

## Execution Interface

There are three modes for controlling robots, with lots of debug introsecption functions.

## Planning Interface

Various functions for Cartesian and sampling-based motion planning

## Remote Control

Wrapper for joystick and interactive marker subscribing, as well as a Rviz GUI plugin

## Trajectory IO

Load and save CSV files for both joint trajectories and cartesian trajectories.
 
