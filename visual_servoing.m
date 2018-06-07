clear
addpath('../class')
close all
clf

global dobot;
global tftree;
tftree = rostf;

% Joint subscriber
jointSub = rossubscriber('/dobot_magician/state','dobot_magician/State',@convertJointAngleToTfCallback);

% Circle subscriber
circleSub = rossubscriber('/hough_circles/circles_float_array','std_msgs/Float32MultiArray');


% Target points in the image. The visual servoing algorithm will try to
% align the current view of the points with these.
desired_points = [179 359 157 343; 117 115 287 287];

dobot = Dobot_sensors('dobot', []);

% Initial pose: We offset them slightly since the IMUs in the dobot joints
% don't like to have the joints at 90 degrees.
q0 = [0; deg2rad(20); deg2rad(20)];
dobot.moveRobotToJoint(q0');

% Add the camera - these values should be set based on the camera
% intrinsics being used.
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512], 'name', 'Dobotcamera');

% Gain of the controller
lambda = 0.6;
dobot.PlotRobot(q0);

% Set pose of camera with respect to end effector. Can be found using
% sensors_main_eye_on_hand.m
T_EE_cam = [-0.0019   -0.9970   -0.0778    0.1174;
            -0.0732   -0.0775    0.9943   -0.0103;
            -0.9973    0.0076   -0.0728   -0.0083;
                 0         0         0    1.0000];
cam.T = dobot.fkineToEndJoint3(q0) * T_EE_cam;

 while true        
    % Get the coordinates of the circles in the image.
    circle_msg = receive(circleSub);
    circle_array = circle_msg.Data;

    % Keep looping until there are 4 circles visible in the image.
    while (size(circle_array,1) ~= 8)
        circle_msg = receive(circleSub);
        circle_array = circle_msg.Data;
        disp('Cant find 4 circles');
    end

    % Associate observed circles with the desired circle positions. We
    % simply divide the viewed circles into 4 quadrants and match them with 
    % the target circle in the same quadrant. This will break down if the 
    % square shape is rotated 45 degrees and also means that 90 degree
    % rotations of the real pattern is considered the same.
    x_avg = sum(circle_array(1:4)) / 4;
    y_avg = sum(circle_array(5:8)) / 4;

    for i=1:4
        p = [circle_array(i); circle_array(i+4)];
        if (p(1) < x_avg && p(2) < y_avg)
            point_1 = p;
        elseif (p(1) > x_avg && p(2) < y_avg)
            point_2 = p;
        elseif (p(1) < x_avg && p(2) > y_avg)
            point_3 = p;
        elseif (p(1) > x_avg && p(2) > y_avg)
            point_4 = p;
        else
            error("Incorrect data association");
        end
    end

    uv = [point_1 point_2 point_3 point_4];

    % Compute the difference between the desired and actual circle
    % coordinates as a column vector.
    e = desired_points-uv;   
    e = e(:);

    % Get the most recent joint angles from the robot and use forward
    % kinematics to calculate the current height of the camera. Since the
    % robot end effector is always pointing down, we can use this height as
    % a depth estimate. We also take into account the height of the markers
    % relative to the base of the robot (i.e. if they're sitting on a book)
    joint_angles = receive(jointSub);
    q_current = joint_angles.JointAngles(1:3);
    T_current = dobot.fkineToEndJoint3(q_current) * T_EE_cam;
    marker_height = 0.01;
    depth = T_current(3,4)-marker_height;

    % Calculate the feature jacobian and since the dobot is 3DoF, we only
    % take the first 3 columns.
    J = cam.visjac_p(uv, depth);
    J = J(:,1:3);

    % Compute the velocity of the camera in the camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    
    % Compute the robot's Jacobian in the end effector frame and find its
    % inverse
    J2 = dobot.robot.jacobn(q0);
    J2 = J2(1:3,1:3);
    Jinv = pinv(J2);
    
    % Since the jacobian is in the end effector frame and the velocity we
    % have is in the camera frame, we convert the velocity into the end
    % effector frame.
    T_ = rotz(-pi/2)*rotx(pi/2);
    v = T_' * v;

    % Get the joint velocities from the end effector velocity.
    qp = Jinv*v;
    
    % Discretise the velocity into a distance by multipling by a constant
    % and add it to the current joint angles. 
    q = q0 + qp * 0.5;

    % Send joint commands to robot. The pause is added due to limitations
    % of the dobot, other robots may not require the pause.
    dobot.moveRobotToJoint(q');
    pause(0.05);

    % Update the pose of the camera and the current joint angles of the
    % robot.
    cam.T = dobot.fkineToEndJoint3(q) * T_EE_cam;
    q0 = q;
end


% Callback which converts joint angles into a TF tree. This requires
% knowledge of the robot being used. Here we use a dobot, however, it can
% be modified to use any robot by changing the forward kinematics.
function convertJointAngleToTfCallback(src, msg)
    global dobot;
    global tftree;
    
    q = msg.JointAngles(1:3);
    joint_poses = dobot.getRelativeJointPoses(q);
    joint_names = {'dobot_base', 'dobot_joint_1', 'dobot_joint_2', ...
                   'dobot_joint_3', 'dobot_end'};
                  
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.Header.Stamp = rostime('now');
    tfStampedMsg.Transform.Rotation.W = 1;
    tfStampedMsg.ChildFrameId = joint_names{1};
    tfStampedMsg.Header.FrameId = 'world';
    sendTransform(tftree, tfStampedMsg);
    
    tfStampedMsg2 = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg2.Header.Stamp = rostime('now');
    tfStampedMsg2.Transform.Rotation.W = 0.4604;
    tfStampedMsg2.Transform.Rotation.X = -0.5358;
    tfStampedMsg2.Transform.Rotation.Y = 0.4993 ;
    tfStampedMsg2.Transform.Rotation.Z = 0.5016;
    
    tfStampedMsg2.Transform.Translation.X = 0.1174;
    tfStampedMsg2.Transform.Translation.Y = -0.0103;
    tfStampedMsg2.Transform.Translation.Z = -0.0083;
    tfStampedMsg2.ChildFrameId = 'usb_cam';
    tfStampedMsg2.Header.FrameId = 'dobot_end';
    sendTransform(tftree, tfStampedMsg2);
            
    for i=1:size(joint_poses,3)
        tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg.ChildFrameId = joint_names{i+1};
        tfStampedMsg.Header.FrameId = joint_names{i};

        tfStampedMsg.Transform.Translation.X = joint_poses(1,4,i);
        tfStampedMsg.Transform.Translation.Y = joint_poses(2,4,i);
        tfStampedMsg.Transform.Translation.Z = joint_poses(3,4,i);
        
        r = joint_poses(1:3,1:3,i);
        quatrot = rotm2quat(r);
        tfStampedMsg.Transform.Rotation.W = quatrot(1);
        tfStampedMsg.Transform.Rotation.X = quatrot(2);
        tfStampedMsg.Transform.Rotation.Y = quatrot(3);
        tfStampedMsg.Transform.Rotation.Z = quatrot(4);
        tfStampedMsg.Header.Stamp = rostime('now');
        
        sendTransform(tftree, tfStampedMsg);
    end
end