# Mini Project 1
 This is Peter Wu's 15464 Mini Project 1, a basic IK solver with Cyclic-Coordinate Descent(CCD) and Forward And Backward Reaching Inverse Kinematics(FABRIK).
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
  Scene*-CCD.mp4 scenes implemented by CCD
  Scene*-FAB.mp4 scenes implemented by FABRIK
  
## Running
 Just directly modify the .csv files and run the .py files
