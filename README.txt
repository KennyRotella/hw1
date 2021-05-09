The file with instructions must be a .txt file and must contain (at least):

- Student ID ("Person Code"), name and surname of all team members
    10634706 Massimo Del Tufo
    10716950 Praneeth Reddy Arikatla
    
- small description of the files inside the archive
    parameters.cfg             configuration file for dynamic parameter to select integration method
    hw1.launch                 launch file set paramiter for init pose and run nodes
    Odom.msg                   custom message to pusblish odometry and integration method infos
    chiparam_approx.cpp        approximates chi parameter (for apparent baseline), it takes the mean value of all reasonable values
    filter_msg.cpp             synchronizes motor messages and convert them rpm -> rad/s, pusblish topic /scout_speeds
    gearratio_approx.cpp       approximates gearration taking the mean of all reasonable values (between 1/40 1/35)
    odometry.cpp               computes pose ("euler" "rk"), sets init pose, publish topics (/custom_message, /odom_approx) subs to /scout_speeds, service server ("set_odometry")
    odometry_tf.cpp            broadcasts TF robot child of odom frame
    reset_od_client.cpp        client to reset (0,0,0) pose
    set_od_client.cpp          client to set (x,y,th) pose
    SetOdometry.srv            declare service parameters

- name and meaning of the ROS parameters
    /initial_pose                    initial pose of the scout
    /odometry/integration_method     dynamic param to choose between integration methods "Euler" 0 or "Runge Kutta" 1

- structure of the TF tree
    /odom
        /robot

- structure of the custom message
    nav_msgs/Odometry odom
    std_msgs/String method

- description of how to start/use the nodes
    roslaunch hw1 hw1.launch                                                    to launch all the needed nodes
    rosrun dynamic_reconfigure dynparam set /odometry integration_method 0|1    "0" euler "1" rk
    rosrun hw1 set_od_client x y th                                             set robot's pose to a (x,y,th) position
    rosrun hw1 reset_od_client                                                  reset robot's pose to (0,0,0) position
    rosrun hw1 chiparam_approx                                                  computes the mean of a sequence of values derived from bags' topic /scout_odom
    rosrun hw1 gearratio_approx                                                 computes the mean of a sequence of values derived from bags' topic /scout_odom

- info you think are important/interesting
    for the computation of gear ratio and apparent baseline I've used the method suggested by prof. Cudrano, all values resulting from the computation (in a reasonable interval) were used to compute a mean value,
    so every value outside a given range were not considered, [1/40, 1/35] for gear ratio, [1.5, 2] for chi parameter. To visualize with 'rviz' use 'odom' as fixed frame.
