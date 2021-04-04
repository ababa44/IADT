classdef LineTraceCarController < CarController
    %LINETRACECARCONTROLLER ライントレースコントローラ
    %   車両モデルとは無関係に参照軌道上を一定車速でトレースする
    
    methods
        function obj = LineTraceCarController(path)
            % オブジェクトの生成
            obj@CarController(path);
        end
        function [vx,vy,omega,direction] = calculateCarVelocity(obj,dt)
            % 与えられた速さで，現在位置から最も近いパス上の線分と同じ方向に進む際の速度ベクトルを計算する
            car = obj.parent;
            v = obj.vehicle_speed;
            %参照軌道上の最近傍点を算出し，接線ベクトルを算出する．
            [~,p1,p2] = calculateShortestDistancePath(obj,getPosition(car)');
            vector = obj.path(:,p2)-obj.path(:,p1);
            %出力
            vx = v*vector(1)/norm(vector);
            vy = v*vector(2)/norm(vector);
            omega = (atan2(vy,vx)-atan2(car.vy,car.vx))/dt;
            direction = sign(v);
        end
    end
    
end

