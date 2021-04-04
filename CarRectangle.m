classdef CarRectangle < MyRectangle
    %CARRECTANGLE �����`�̎ԗ��N���X
    %   xy���ʏ�ł̎ԗ��̓����i���x�E�p���x�j�𐧌�
    
    properties
        vx %x���������x
        vy %y���������x
        omega %�p���x
        direction CarGear %�i�s����
        
        controller CarController %�ԗ��̃R���g���[��
    end
    
    methods
        function obj = CarRectangle(x, y, theta, vx, vy, omega, car_width, car_length)
            % �I�u�W�F�N�g�̐���
            obj@MyRectangle(x, y, car_length, car_width, theta)
            obj.vx = vx;
            obj.vy = vy;
            obj.omega = omega;
            obj.direction = 'neutral';
        end
        function obj = setController(obj, controller)
            % ����p�̃R���g���[����ݒ�
            obj.controller = controller;
            obj.controller.parent = obj;
        end
        
        function obj = updateDirection(obj, direction)
            % �ԗ��̐i�s�������X�V
            obj.direction = direction;
        end
        function obj = updateVelocity(obj, vx, vy, omega)
            % �ԗ��̑��x�E�p���x���X�V
            obj.vx = vx;
            obj.vy = vy;
            obj.omega = omega;
        end
        function obj = updatePosition(obj, dt)
            % �ԗ��̈ʒu�E�p�x���X�V
            fd = f_rk4(@(x,u) u, dt);
            obj.x = fd(obj.x,obj.vx);
            obj.y = fd(obj.y,obj.vy);
            obj.theta = fd(obj.theta,obj.omega);
        end

        function v = getVelocity(obj)
            % �ԗ��̑��x���擾
            v = [obj.vx;obj.vy];
        end
        
        function plot(obj,varargin)
            % �ԗ��̕\��
            % �ԗ��̌`��E�ԗ��̌����E�Q�ƋO����\������
            for k = 1:numel(obj)
                x = obj(k).x;
                y = obj(k).y;
                v_theta = obj(k).theta;
                plot@MyRectangle(obj(k),varargin{:})
                plot([x, x+cos(v_theta)],[y, y+sin(v_theta)],'Color','k')
                if ~isempty(obj(k).controller)
                    plot(obj(k).controller,'LineStyle','-','LineWidth',2)
                end
            end
        end
    end
end

