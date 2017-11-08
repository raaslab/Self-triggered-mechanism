  animate = 0;
  posetopics{1} = '/robot_0/p3dx/base_pose_ground_truth';
   posetopics{2} = '/robot_1/p3dx/base_pose_ground_truth';
   posetopics{3} = '/robot_2/p3dx/base_pose_ground_truth';
   posetopics{4} = '/robot_3/p3dx/base_pose_ground_truth';
   posetopics{5} = '/robot_4/p3dx/base_pose_ground_truth';
% 
   posetopics{6} = '/robot_5/RosAria/pose';
   posetopics{7} = '/robot_6/RosAria/pose';


plot_rosbag('/home/tulipa/2017-10-31-22-48-56.bag', posetopics, true)