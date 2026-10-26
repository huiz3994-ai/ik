# This is the force as estimated from the applied torques as well as the current
# end effector position of the robot arm   
# 两种力反馈方式，反馈力，限制位置，反馈力最大为3.3N                        
geometry_msgs/Vector3  force                                                                  
geometry_msgs/Vector3  position 
bool[] lock 