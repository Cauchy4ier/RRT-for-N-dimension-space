# RRT-for-N-dimension-space  
A RRT algorithm written in Python 2.7 works for N dimension configuration space. N is defined by the size of starting point and the objective. Here we have a simply 2D planar example shown as follow. Inheritation is a bit different between Python2 and Python3, so it might cause some confusion if you run the codes in Python3.  
Before testing the RRT alogorithm, install all the nessary Python libraries listed in the txt file.  
`pip install -r requirements.txt`  
In the *test.test_rrt* you can freely change the starting point and the objective point. Use `nose2 test.test_rrt` to test all the cases.   
Want to acheive a more directly seen path? Run `nose2 test.test_planar_rrt` for a 2D visualization.  
![](https://github.com/Cauchy4ier/RRT-for-N-dimension-space/blob/main/example_planar_rrt.png)



