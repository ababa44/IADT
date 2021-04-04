classdef PIDCarController < CarController
    %PIDCarController PID�R���g���[��
    %   PID�����Ƃɉ��΍��ƕ��ʕ΍���0�ɂ�������Ɏ��Ԏp������]������
    %   ���x�̐�Βl�͈��Ƃ���
    
    properties
        KP = [1.5,1.5]; % P�Q�C��
        KD = [0.1,0.1]; % D�Q�C��
        KI = [0.7,0.7]; % I�Q�C��
        integral_e_v = [0;0]; %�덷�̐ϕ��l
        previous_e_v = [0;0]; %�덷�̑O��l�i�����p�j
    end
    
    methods
        function obj = PIDCarController(path)
            % �I�u�W�F�N�g�̐���
            obj@CarController(path);
        end
        
        function delta = calculateSteeringAngle(obj,dt)
            %PID�ɂ��O�֓]�Ǌp�w�ߒl�̌v�Z
            car = obj.parent;
            x_vcl = car.x;
            y_vcl = car.y;
            theta_vcl = car.theta;
            xk = [x_vcl,y_vcl,theta_vcl]';

            %�덷�x�N�g��
            e_v = calculateLateralAndOrientationError(obj,xk);
            %�덷�x�N�g���̐ϕ�
            obj.integral_e_v = obj.integral_e_v+e_v*dt;
            %�덷�x�N�g���̔���
            de_v = (e_v-obj.previous_e_v)/dt;
            obj.previous_e_v = e_v;
            %PID�ɂ�鑀�Ǔ���
            delta = -(obj.KP*e_v+obj.KI*obj.integral_e_v+obj.KD*de_v);
        end
    end
end

