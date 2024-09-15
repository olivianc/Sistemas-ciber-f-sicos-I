setenv('ROS_DOMAIN_ID','0');
ros2('topic','list');
node= ros2node('/imu_node');
imuPu= ros2subscriber(node,'imu', 'sensor_msgs/Imu');
serverIP1='10.42.0.196';
serverPort=80;
tcpcobj=tcpclient(serverIP1,serverPort);
pause(0.1);
while true
    [msg, status, statusText]=receive(imuPu); 
    pause(0.1);
   x= msg.linear_acceleration.x;
   y= msg.linear_acceleration.y;
   z= msg.angular_velocity.z;
    disp([x,y,z]);
    writeline(tcpcobj, num2str([x,y,z]));
end 
clear node