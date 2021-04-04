classdef PurePursuitCarController < CarController
    %PurePursuitCarController PurePursuit�R���g���[��
    %   PurePursuit�����ƂɎ��Ԏp������]������
    %   ���x�̐�Βl�͈��Ƃ���
    
    properties
        gazepoint_length = 3;%Lookahead Distance
    end
    
    methods
        function obj = PurePursuitCarController(path)
            % �I�u�W�F�N�g�̐���
            obj@CarController(path);
        end

        function delta = calculateSteeringAngle(obj,dt)
            %PurePursuit�ɂ��O�֓]�Ǌp�w�ߒl�̌v�Z
            car = obj.parent;
            x_vcl = car.x;
            y_vcl = car.y;
            theta_vcl = car.theta;
            %Lookahead Distance�O���̓_�̎擾
            p_gaze_direct = getPosition(car)' + obj.gazepoint_length*[cos(theta_vcl),sin(theta_vcl)];
            %�p�X��̐��������
            [~,g1,g2] = calculateShortestDistancePath(obj,p_gaze_direct);
            %������̒����_�����
            [~,q] = obj.calculatePointLinesegmentDistance(p_gaze_direct',obj.path(:,g1),obj.path(:,g2));
            x_gaze = q(1);
            y_gaze = q(2);
            %���ݕ��ʂƂ̒����_�̕��ʍ�
            alpha_vcl = -theta_vcl + atan2(y_gaze-y_vcl,x_gaze-x_vcl);
            % ��]���a�̌v�Z
            r_vcl = obj.gazepoint_length/sin(alpha_vcl)/2;
            %�ŏ���]���a�̐ݒ�
            %r_vcl(abs(r_vcl)<3)=3*sign(r_vcl);
            %PurePursuit�ɂ�鑀�Ǔ���
            delta = atan(obj.vehicle_length/r_vcl);
        end
    end
end

