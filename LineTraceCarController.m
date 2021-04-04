classdef LineTraceCarController < CarController
    %LINETRACECARCONTROLLER ���C���g���[�X�R���g���[��
    %   �ԗ����f���Ƃ͖��֌W�ɎQ�ƋO��������ԑ��Ńg���[�X����
    
    methods
        function obj = LineTraceCarController(path)
            % �I�u�W�F�N�g�̐���
            obj@CarController(path);
        end
        function [vx,vy,omega,direction] = calculateCarVelocity(obj,dt)
            % �^����ꂽ�����ŁC���݈ʒu����ł��߂��p�X��̐����Ɠ��������ɐi�ލۂ̑��x�x�N�g�����v�Z����
            car = obj.parent;
            v = obj.vehicle_speed;
            %�Q�ƋO����̍ŋߖT�_���Z�o���C�ڐ��x�N�g�����Z�o����D
            [~,p1,p2] = calculateShortestDistancePath(obj,getPosition(car)');
            vector = obj.path(:,p2)-obj.path(:,p1);
            %�o��
            vx = v*vector(1)/norm(vector);
            vy = v*vector(2)/norm(vector);
            omega = (atan2(vy,vx)-atan2(car.vy,car.vx))/dt;
            direction = sign(v);
        end
    end
    
end

