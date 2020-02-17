# Mini Project 1
 This is Peter Wu's 15464 Mini Project 1, a basic IK solver with Cyclic-Coordinate Descent(CCD) and Forward And Backward Reaching Inverse Kinematics(FABRIK). The github link is https://github.com/Wuhao1627003/15464-Mini-1.
 In 15462, I already implemented Jacobian Transpose and Jacobian Pseudo-Inverse methods, so here I add 2 more methods into my toolbox: CCD and FABRIK.
 
 For this project, I started from scratch and used basic matplotlib 3d plotting to plot joints and chains. My main goal is to try several scenarios and compare how good these two methods are, in terms of runtime, no. iterations, and naturalness.
 For CCD, I implemented the rotations with quaternions, allowed for multiple constraints by greedy heurestic (which doesn't always work, I must admit), and allowed for tree input. For FABRIK, I only implemented the basic version of moving the end effector for a single chain.
 
## Files
 Code:
  ccd.py parameters: boolean(VIS) whether axis is visible; float(PAUSETIME) pause time for each frame; string(mode) "list" for chain input, "tree" for tree input; float_tuple_array(queries) each element is (indexOfJoint, newx, newy, newz, threshold)
  fabrik.py parameters: boolean(VIS) whether axis is visible; float(PAUSETIME) pause time for each frame; float_tuple(query) where query is (newx, newy, newz, threshold)
  
 Data:
  list.csv each row gives (x, y, z) coordinate of a joint consecutively, representing a chain, with 0th row being the root
  tree.csv each row gives (parent, x, y, z) coordinate of a joint, representing a multi-chain, with 0th row being the root
  
 Video:
  Demo.mp4 sum of all scenes

  Scene*-CCD_x265.mp4 scenes implemented by CCD
  Scene*-FAB_x265.mp4 scenes implemented by FABRIK

  0	  0	0
  1	  0	0
  3	  0	0
  4	  0	0
  5	  0	0
  8	  0	0
  8.5	0	0
  12	0	0
  13	0	0
  17	0	0
  20	0	0
  25	0	0
  30	0	0
  1: reachable 12, 15, 10, .006
  2: reachable 12, 15, 10, .001
  3: reachable 6, 3, 4, .001
  4: reachable 6, 7, 9, .01
  5: unreachable 10, 10, 30, .1

  0	  0	   0
  1	  1	   0
  3	  2	   7
  4	  3	   1
  5	  -1	 2
  8	  4	   8
  8.5	6	   -1
  12	-3.2 5
  13	6	   10
  6: reachable -10, 11, -2, .01
  7: reachable 11, 20, 10, .01
  8: reachable 11, 30, 13, .1

  9: reachable [[3, 6, 2, 1, .01], [6, 10, 10, 13, .1]]
  10: reachable [[3, 6, 2, 1, .01], [6, 8, 10, 13, .1], [8, 10, 11, 5, 1]]

  0	0	  0	  0
  0	1	  1	  0
  1	0.5	2	  1
  2	-1	2	  -1
  2	1.5	4	  0
  1	3	  1.5	1
  5	3	  3	  2
  6	3	  5	  3
  6	4	  5	  2
  5	6	  0.6	0
  11: reachable [6, -4, -2, 2, .6], [9, -3, 2, 1, .01]

  Tutorial:
    *.pdf algorithm descriptions

  
## Running
 Just directly modify the .csv files and run the .py files
