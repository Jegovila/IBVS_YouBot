classdef Bot_youBot < handle
    properties (Access = private)
        CoppeliaSim = [];
        ClientID = [];

        youBot = [];
        Joints = [];
        Gripper = [];
        Motors = [];

        camera = [];

        wl = -2.5*ones(4,1);
        wu = 2.5*ones(4,1);

        qp_l = deg2rad(-[5 5 5 5 5]');
        qp_u = deg2rad(+[5 5 5 5 5]');
    end
    
    methods
        function obj = Bot_youBot()
            obj.CoppeliaSim = remApi('remoteApi');
            obj.CoppeliaSim.simxFinish(-1);
            disp('Connecting to robot....')
            obj.ClientID = obj.CoppeliaSim.simxStart('127.0.0.1',19999,true,true,5000,5); 
            
            if obj.ClientID==0
                [~,obj.youBot] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Motors(1)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/rollingJoint_fl',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Motors(2)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/rollingJoint_fr',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Motors(3)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/rollingJoint_rl',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Motors(4)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/rollingJoint_rr',obj.CoppeliaSim.simx_opmode_oneshot_wait);

                [~,obj.Joints(1)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint0',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(2)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint1',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(3)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint2',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(4)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint3',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Joints(5)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotArmJoint4',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Gripper(1)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotGripperJoint1',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                [~,obj.Gripper(2)] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/youBotGripperJoint2',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                
                [~,obj.camera] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/youBot/kinect/rgb',obj.CoppeliaSim.simx_opmode_oneshot_wait);
                pause(1)

                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper(2),-0.01,obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper(1),0.01,obj.CoppeliaSim.simx_opmode_streaming);

                disp(' done.')
            else
                error(' robot not connected...')
            end
        end
        
        function obj = Gripper_Command (obj,action) % action = 0 para cerrar, action = 1 para abrir
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                if action == 0
                    obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper(1),-0.05,obj.CoppeliaSim.simx_opmode_streaming);
                    obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper(2),0.05,obj.CoppeliaSim.simx_opmode_streaming);
                elseif action == 1
                    obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper(1),0.05,obj.CoppeliaSim.simx_opmode_streaming);
                    obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Gripper(2),-0.05,obj.CoppeliaSim.simx_opmode_streaming);
                end
            else  
                error(' connection lost...')
            end
        end
        
        function q = Get_Arm_Position (obj)
            aux = ones(5,1);
            q = zeros(5,1);

            while sum(aux)~=0
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [aux(1),q(1)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(1),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(2),q(2)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(2),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(3),q(3)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(3),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(4),q(4)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(4),obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(5),q(5)] = obj.CoppeliaSim.simxGetJointPosition(obj.ClientID,obj.Joints(5),obj.CoppeliaSim.simx_opmode_streaming);
                else
                    error(' connection lost...')
                end
            end
        end

        function p = Get_Platform_Pose (obj)
            aux = ones(2,1);

            while sum(aux)~=0
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [aux(1),position] = obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID,obj.youBot,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(2),orientation] = obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID,obj.youBot,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    
                    p = double([position(1) position(2) orientation(3)]');
                else
                    error(' connection lost...')
                end
            end
        end

        function [position] = Test (obj)
            aux = ones(2,1);

            while sum(aux)~=0
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [aux(1),position] = obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID,obj.camera,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    [aux(2),orientation] = obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID,obj.youBot,-1,obj.CoppeliaSim.simx_opmode_streaming);
                    
                    position = -position;
                else
                    error(' connection lost...')
                end
            end
        end

        function obj = Set_Joint_Velocity (obj,qp)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                qp = max(qp,obj.qp_l); qp = min(qp,obj.qp_u);
                
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(1),qp(1),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(2),qp(2),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(3),qp(3),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(4),qp(4),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(5),qp(5),obj.CoppeliaSim.simx_opmode_streaming);
            else
                error(' connection lost...')
            end
        end

        function obj = Set_Joint_Position (obj,qp)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                qp = max(qp,obj.qp_l); qp = min(qp,obj.qp_u);
                
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(1),qp(1),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(2),qp(2),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(3),qp(3),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(4),qp(4),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Joints(5),qp(5),obj.CoppeliaSim.simx_opmode_streaming);
            else
                error(' connection lost...')
            end
        end
        
        function obj = Set_Platform_Velocity (obj,w)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                w = max(w,obj.wl); w = min(w,obj.wu);
                
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Motors(1),-w(1),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Motors(2),-w(2),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Motors(3),-w(3),obj.CoppeliaSim.simx_opmode_streaming);
                obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.Motors(4),-w(4),obj.CoppeliaSim.simx_opmode_streaming);
            else
                error(' connection lost...')
            end
        end
        
       function img = Get_Image (obj)
            aux = 1;
            
            while aux~=0
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)==1
                    [aux,~,img] = obj.CoppeliaSim.simxGetVisionSensorImage2(obj.ClientID,obj.camera,0,obj.CoppeliaSim.simx_opmode_streaming);
                else
                    error(' connection lost...')
                end
            end
        end

        function s = Connection(obj)
            s = obj.CoppeliaSim.simxGetConnectionId(obj.ClientID);
        end

        function Stop_Simulation (obj)
            obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);
        end
    end
end
