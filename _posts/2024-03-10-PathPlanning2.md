---
layout: post
title: "Trajectory generation for autonomous robots - PART 2: Motion profiling"
date: 2024-03-10 10:00:00 +0300
categories: [Robotics, Trajectory Generation]
tags: [robotics, path planning, motion profile, first tech challenge]
image: simulator.gif
math: true
img_path: /assets/images/MotionProfileImages
---

## Introduction

In a previous post, we discussed how to create arbitrary spline curves and reparametrize them with displacement. In this post, we'll tackle how to make a robot follow such a path.

In particular, *we need to describe the position, velocity and acceleration of the robot at any point in time during the trajectory*. A **motion profile** is exactly what we need.

Here is a very simple motion profile:


The algorithm to generate path-based, kinematic-constrained, time-optimal motion profiles for mobile robots is taken from this paper, albeit with significant simplifications:   
`Lau, Boris & Sprunk, Christoph & Burgard, Wolfram. (2009). Kinodynamic Motion Planning for Mobile Robots Using Splines. 2009 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS 2009. 2427-2433. 10.1109/IROS.2009.5354805.`


## Motion profile generation 

### Robot constraints
Each robot has some kinematic constraints. We will use the following ones here:
- Maximum translational velocity
- Maximum translational acceleration
- Maximum angular velocity 
- Maximum angular acceleration

Theoretically the algorithm supports arbitrary constraints to be defined and enforced in the following steps. I will only discuss constant (throughout the path) constraints here.

### First pass: maximum translation velocity
The first step is to construct a velocity profile made up of discrete `planning points` distributed on the arc length of the path, with initial velocity set to the maximum translational velocity defined before.
### Second pass: slowdown near high curvature

### Third pass: maximum translation acceleration

### Fourth pass: user-defined constraint satisfaction

### Final profile

