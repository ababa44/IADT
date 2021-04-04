classdef CarRectangle < MyRectangle
    %CARRECTANGLE 長方形の車両クラス
    %   xy平面上での車両の動き（速度・角速度）を制御
    
    properties
        vx %x軸方向速度
        vy %y軸方向速度
        omega %角速度
        direction CarGear %進行方向
        
        controller CarController %車両のコントローラ
    end
    
    methods
        function obj = CarRectangle(x, y, theta, vx, vy, omega, car_width, car_length)
            % オブジェクトの生成
            obj@MyRectangle(x, y, car_length, car_width, theta)
            obj.vx = vx;
            obj.vy = vy;
            obj.omega = omega;
            obj.direction = 'neutral';
        end
        function obj = setController(obj, controller)
            % 制御用のコントローラを設定
            obj.controller = controller;
            obj.controller.parent = obj;
        end
        
        function obj = updateDirection(obj, direction)
            % 車両の進行方向を更新
            obj.direction = direction;
        end
        function obj = updateVelocity(obj, vx, vy, omega)
            % 車両の速度・角速度を更新
            obj.vx = vx;
            obj.vy = vy;
            obj.omega = omega;
        end
        function obj = updatePosition(obj, dt)
            % 車両の位置・角度を更新
            fd = f_rk4(@(x,u) u, dt);
            obj.x = fd(obj.x,obj.vx);
            obj.y = fd(obj.y,obj.vy);
            obj.theta = fd(obj.theta,obj.omega);
        end

        function v = getVelocity(obj)
            % 車両の速度を取得
            v = [obj.vx;obj.vy];
        end
        
        function plot(obj,varargin)
            % 車両の表示
            % 車両の形状・車両の向き・参照軌道を表示する
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

