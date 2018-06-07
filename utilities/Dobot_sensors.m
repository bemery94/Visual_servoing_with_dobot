classdef Dobot_sensors < handle
    properties
        % Horizontal and vertical offset respectively from the 3rd joint of
        % the dobot to the point of the vacuum gripper. Taken looking at it
        % from the side view.
        j3_to_gripper_offset = [0.06 -0.0555]'; 
        
        % Link lengths used in the DH Params.
        base_to_j1 = 0.0784;
        l1 = 0.135;
        l2 = 0.147;
        
        T_base_to_world = transl(0, 0, 0.057);
        
        steps = 2;
        
        workspace;
        robot;
        ee_h;
        joint_sub;
        emergency_stop;
    end
    
    methods
        % Initialize UR3 robot
        function self = Dobot_sensors(name, workspace)
            self.robot.base = self.T_base_to_world;
            
            if (isequal(size(workspace), [1, 6]))
                self.workspace = workspace;
            end
            
            self.CreateRobot(name);
            
            self.emergency_stop = rossubscriber('/dobot_magician/emergency_stop','std_msgs/Bool', @self.StopCallback);
            self.joint_sub = rossubscriber('/dobot_magician/state','dobot_magician/State');
        end
        
        function CreateRobot(self, name)
            % Links of the robot using DH parameters
            L1 = Link('offset',0,'d',0.0784,'a',0,'alpha',-pi/2,'qlim',[-pi,pi]);
            L2 = Link('offset',-pi/2,'d',0,'a',0.1393,'alpha',0,'qlim',[0,deg2rad(85)]);
            L3 = Link('offset',pi/2,'d',0,'a',0.16198,'alpha',0,'qlim',[deg2rad(-10),deg2rad(95)]);
            
            self.robot = SerialLink([L1, L2, L3], 'name', name);
                          
            q = [0 0 0];
            self.PlotRobot(q);
        end
                            
        function self = PlotRobot(self, q)
            q_model = self.converQRealToQModel(q);
            
            % Plot the robot from the DH params
            self.robot.plot(q_model,'workspace',self.workspace);
            hold on
            
            % Given a set of joint angles, find the location of the gripper
            T_end_effector = self.fkineToEndEffector(q);
            try
                delete(self.ee_h);
            end
            
            % Plot the gripper. This can be replaced by a mesh.
            self.ee_h = trplot(T_end_effector,'length',0.1);
        end
        
        
        function q_real = ikineToPoint(self, p)
            % This function finds the joint angles to pick up an object at 
            % coords p. p is a 3-vector.
            assert(isequal(size(p), [3 1]));

            % Find the direction from the base of the robot to the point
            theta_1 = atan2(p(2), p(1));
    
            
            % Find the position of the 3rd joint with respect to the base
            % by subtracting the offset of the gripper wrt. the 3rd joint.
            t_base_j3 = [p(1) - self.j3_to_gripper_offset(1) * cos(theta_1);
                         p(2) - self.j3_to_gripper_offset(1) * sin(theta_1);
                         p(3) - self.j3_to_gripper_offset(2)];
                     
            T_base_j3 = rt2tr(eye(3), t_base_j3);
            
            % Find the joint angles to reach joint 3 position found above
            % while masking rotation and finding a solution within joint 
            % limits.
            [q_model, err] = self.robot.ikcon(T_base_j3, true);
            disp(['Optim error = ', num2str(err)]);
            
            q_real = self.converQModelToQReal(q_model);
        end
        
        function T_end_point = fkineToEndEffector(self, q)
            q_model = self.converQRealToQModel(q);
            tr = zeros(4,4,self.robot.n+1);
            tr(:,:,1) = self.robot.base;
            L = self.robot.links;
            for i = 1 : self.robot.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q_model(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            T_end_joint = tr(:,:,4);
            
            end_point = [T_end_joint(1,4) + self.j3_to_gripper_offset(1) * cos(q_model(1));
                         T_end_joint(2,4) + self.j3_to_gripper_offset(1) * sin(q_model(1));
                         T_end_joint(3,4) + self.j3_to_gripper_offset(2)];
            
            T_end_point = rt2tr(rotz(q(1)) * rotx(pi), end_point);
        end
        
        function T_end_joint = fkineToEndJoint3(self, q)
            q_model = self.converQRealToQModel(q);
            tr = zeros(4,4,self.robot.n+1);
            tr(:,:,1) = self.robot.base;
            L = self.robot.links;
            for i = 1 : self.robot.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q_model(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            T_end_joint = tr(:,:,4);
            
        end
        
        function joint_poses = getRelativeJointPoses(self, q)
            q_model = self.converQRealToQModel(q);
            joint_poses = zeros(4,4,self.robot.n+1);
            joint_poses(:,:,1) = self.robot.base;
            L = self.robot.links;
            
            for i = 1 : self.robot.n
                joint_poses(:,:,i+1) =  ...
                    trotz(q_model(i)+L(i).offset) * transl(0,0,L(i).d) * ...
                    transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
        end
        
        function joint_poses= getGlobalJointPoses(self, q)
            q_model = self.converQRealToQModel(q);
            joint_poses = zeros(4,4,self.robot.n+1);
            joint_poses(:,:,1) = self.robot.base;
            L = self.robot.links;
            
            for i = 1 : self.robot.n
                joint_poses(:,:,i+1) =  joint_poses(:,:,i) * ...
                    trotz(q_model(i)+L(i).offset) * transl(0,0,L(i).d) * ...
                    transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
        end
        
        function robot = GetRobot(self)
            robot = self.robot;
        end    
        
        function q_model = converQRealToQModel(self, q)
            q_model = [q(1) q(2) q(3) - q(2)];
        end

        function q_model = converQModelToQReal(self, q)
            q_model = [q(1) q(2) q(3) + q(2)];
        end
        
        function activatePump(self)
            srv = rossvcclient('/dobot_magician/pump');
            service_msg = rosmessage('dobot_magician/SetPump');
            service_msg.Pump = true;

            try
                srv.call(service_msg,'Timeout',0.01);
            catch
                disp('Catching deactivate pump');
            end
            pause(0.5);
        end
        
        function deactivatePump(self)
            srv = rossvcclient('/dobot_magician/pump');
            service_msg = rosmessage('dobot_magician/SetPump');
            service_msg.Pump = false;

            try
                srv.call(service_msg,'Timeout',0.01);
            catch
                disp('Catching deactivate pump');
            end
            pause(0.5);
        end
        
        function q_real = getModelPos(self)
            q = self.robot.getpos();
            q_real = self.converQModelToQReal(q);
        end
        
        function moveRobotToPoint(self, goal_point)
            assert(isequal(size(goal_point), [3 1]));
            
            q_goal = self.ikineToPoint(goal_point);
            
            srv = rossvcclient('/dobot_magician/joint_angs');
            service_msg = rosmessage('dobot_magician/SetPosAng');
            service_msg.JointAngles = [q_goal 0];
            srv.call(service_msg,'Timeout',10.5);

            self.robot.animate(q_goal);
        end
        
        function moveRobotToJoint(self, goal_joint)
            assert(isequal(size(goal_joint), [1 3]));      
            
            srv = rossvcclient('/dobot_magician/joint_angs');
            service_msg = rosmessage('dobot_magician/SetPosAng');
            service_msg.JointAngles = [goal_joint 0];
            srv.call(service_msg,'Timeout',5.5);

            self.robot.animate(goal_joint);            
        end
        
        function trajectory_goal = getRobotTrajectory(self, goal_point)
            assert(isequal(size(goal_point), [3 1]));
            q_goal = self.ikineToPoint(goal_point);
            
            sim_joint_state = self.getModelPos();
            s = lspb(0,1,self.steps);
            trajectory_goal = nan(self.steps,3);
            for i = 1:self.steps
                trajectory_goal(i,:) = (1-s(i))*sim_joint_state + s(i)*q_goal;
            end
        end
    end
end
