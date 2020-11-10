# RRT-for-N-dimension-space  
A RRT algorithm written in Python 2.7 works for N dimension configuration space. N is defined by the size of starting point and the objective. Here we have a simply 2D planar example shown as follow. Inheritation is a bit different between Python2 and Python3, so it might cause some confusion if you run the codes in Python3.  
Before testing the RRT alogorithm, install all the nessary Python libraries listed in the txt file.  
`pip install -r requirements.txt`  
In the *test.test_rrt* you can freely change the starting point and the objective point. Use `nose2 test.test_rrt` to test all the cases.   
Want to acheive a more directly seen path? Run `nose2 test.test_planar_rrt` for a 2D visualization.  
![](https://github.com/Cauchy4ier/RRT-for-N-dimension-space/blob/main/example_planar_rrt.png)  
What about 6 Dof robotic arm like Ada?  
First, build the workspace  
`cd ros_ws`  
`catkin build`  
Use roscore command and open a new terminal,initiate rviz.  
`source devel/setup.bash`  
`rviz`  
Open another new terminal, run the RRT python code. `python src/adarrt/src/adarrt.py`  
You will see the path coordinates shown on the terminal. If it's your first time running the code, there would nothing shown in rviz. To solve this, find "Add" button on left bottom of rviz. In "by topic" tab, you can see "/dark_markers/simple_trajectories/update/InteractiveMarkers". Add it without hesitation. You can see the Ada arm and the enviroment with a table, a can and two bowls.  
After iterations, follow the instruction appeared on the terminal. Press "Enter" to execute the trajectory, you would see the arm reaches the can and avoids from collision with the bowls and table.  
![](https://github.com/Cauchy4ier/RRT-for-N-dimension-space/blob/main/Screenshot%20from%202020-11-10%2003-01-54.png)





