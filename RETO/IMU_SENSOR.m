setenv('ROS_DOMAIN_ID','0');
ros2('topic','list');
node= ros2node('matlab_cam_node');
imgsub =ros2subscriber(node,'/image_raw','sensor_msgs/Image');
while true
    [imData, imgStatus,imgStatusText]=receive(imgsub);
    img=rosReadImage(imData);
    imshow(img);
    drawnow;
end
clear node