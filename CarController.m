classdef CarController < matlab.mixin.Copyable
    %CARCONTROLLER �ԗ��R���g���[���p�̃x�[�X�N���X
    %   �e�R���g���[���ɋ��ʂŕK�v�ȋ@�\�̎���
    %   �n���h���N���X�Ƃ��Ď������C�R�s�[���\�Ƃ���D
    
    properties
        path %�Q�ƋO��
        parent %�e�i����Ώۂ̎ԗ��j
        
        vehicle_speed = 1; %�ԑ��i�i�s�����ɑ΂��鑬�x�j
        vehicle_length = 2; %�ԗ��̑O��֊Ԃ̒���
    end
    
    methods
        function obj = CarController(path)
            % �I�u�W�F�N�g�̐���
            obj.path = path;
        end
        
        function [vx,vy,omega,direction] = calculateCarVelocity(obj,dt)
            % ���x�E�p���x�E�ԗ��i�s�����̎w�ߒl�̌v�Z
            
            car = obj.parent;
            x_vcl = car.x;
            y_vcl = car.y;
            theta_vcl = car.theta;
            %�O�֓]�Ǌp�̎w�ߒl�̌v�Z
            delta = calculateSteeringAngle(obj,dt);

            %�ԗ����f����p�������x�E�p���x�̌v�Z
            xk = [x_vcl,y_vcl,theta_vcl]';
            xout = calculateVehicleDynamics(obj,xk,delta);
            %�o��
            vx = xout(1);
            vy = xout(2);
            omega = xout(3);
            direction = 'Forward';%�Ƃ肠�����O�i�̂�
        end
        function delta = calculateSteeringAngle(obj,dt)
            % �O�֓]�Ǌp�̎w�ߒl�̌v�Z�i���ۃ��\�b�h�j
            delta = 0;
        end
        function x = calculateVehicleDynamics(obj,x,u)
            % �ԗ����f��
            L = obj.vehicle_length;
            V = obj.vehicle_speed;
            theta = x(3);
            delta = u;
            
            x(1) = V*cos(theta);
            x(2) = V*sin(theta);
            x(3) = V*tan(delta)/L;
        end
        function [distance,index_point1,index_point2] = calculateShortestDistancePath(obj,point)
            % ���݈ʒu����ł��߂��p�X��̐������Z�o����
            % ���̂Ƃ��̋����ƁC�p�X�̉��Ԗڂ̓_�Ɠ_�����񂾐������Z�o
            distance = Inf;
            index_point1 = nan;
            index_point2 = nan;
            for k=1:length(obj.path)-1
                dtmp = obj.calculatePointLinesegmentDistance(point(:),obj.path(:,k+1),obj.path(:,k));
                if dtmp < distance
                    distance = dtmp;
                    index_point1 = k;
                    index_point2 = k+1;
                end
            end
        end
        function error_vector = calculateLateralAndOrientationError(obj,xk)
            %�Q�ƋO���ɑ΂��錻�݈ʒu�E���ʂ̌덷���Z�o����
            
            %���݈ʒu����ł��߂��p�X��̐������Z�o����i�����ɉ��΍����Z�o�j
            [lateral_error,p1_idx,p2_idx] = calculateShortestDistancePath(obj,xk(1:2));
            %�ł��߂��p�X��̐����x�N�g��
            nearest_path_vector = obj.path(:,p2_idx)-obj.path(:,p1_idx);
            %�����Ǝ��Ԃ̈ʒu�֌W�𒲂ׂ�i�O�όv�Z�j
            %�����̎n�_���玩�Ԃ܂ł̃x�N�g����p��
            relative_vcl_vector = xk(1:2)-obj.path(:,p1_idx);
            %�����x�N�g���Ƃ̊O�ς��v�Z
            Z = cross([nearest_path_vector;0],[relative_vcl_vector;0]);
            %z�v�f�̕����ɂ�荶�E�����܂�̂ŁC���΍��ɕ���������
            lateral_error = sign(Z(3))*lateral_error;
            %�����x�N�g���̕���
            theta_path = atan2(nearest_path_vector(2),nearest_path_vector(1));
            %���ʕ΍�
            theta_error = obj.calculateAngleDifference(xk(3),theta_path);

            %�덷�x�N�g��
            error_vector = [lateral_error;theta_error];
        end
        function plot(obj,varargin)
            %�R���g���[���ɐݒ肳�ꂽ�Q�ƋO���̐}��
            line(obj.path(1,:),obj.path(2,:),varargin{:})
        end
    end
    methods (Static)
        function angle_diff = calculateAngleDifference(theta_target,theta_base)
            %�p�x�΍����o�����߂ɕ��f���ŏ���
            complex_target = cos(theta_target)+1i*sin(theta_target);
            complex_base = cos(theta_base)+1i*sin(theta_base);
            %�p�x�΍�
            complex_error = complex_target/complex_base;
            angle_diff = angle(complex_error);
        end
        function [ distance, nearest_point ] = calculatePointLinesegmentDistance( point, end_point1, end_point2 )
            %�_�Ɛ����̋������v�Z�i�����ɂ��̎��̐�����̍ł��߂��_���v�Z�j
            
            % ���ς��g���āC�_p�̐����ɑ΂���ʒu�֌W����肷��
            t = dot(point-end_point1, end_point2-end_point1)/sum((end_point1-end_point2).^2);
            if t < 0
                % �_p�̈ʒu���C�����̒[�_p1�����O���̏ꍇ
                nearest_point = end_point1;
            elseif t > 1
                % �_p�̈ʒu���C�����̒[�_p2�����O���̏ꍇ
                nearest_point = end_point2;
            else
                % �_p�̈ʒu���C�����̓����̏ꍇ
                nearest_point = end_point1+t*(end_point2-end_point1);
            end
            distance = norm(point-nearest_point);
        end
    end
end

