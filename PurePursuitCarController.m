classdef PurePursuitCarController < CarController
    %PurePursuitCarController PurePursuitコントローラ
    %   PurePursuitをもとに自車姿勢を回転させる
    %   速度の絶対値は一定とする
    
    properties
        gazepoint_length = 3;%Lookahead Distance
    end
    
    methods
        function obj = PurePursuitCarController(path)
            % オブジェクトの生成
            obj@CarController(path);
        end

        function delta = calculateSteeringAngle(obj,dt)
            %PurePursuitによる前輪転舵角指令値の計算
            car = obj.parent;
            x_vcl = car.x;
            y_vcl = car.y;
            theta_vcl = car.theta;
            %Lookahead Distance前方の点の取得
            p_gaze_direct = getPosition(car)' + obj.gazepoint_length*[cos(theta_vcl),sin(theta_vcl)];
            %パス上の線分を特定
            [~,g1,g2] = calculateShortestDistancePath(obj,p_gaze_direct);
            %線分上の注視点を特定
            [~,q] = obj.calculatePointLinesegmentDistance(p_gaze_direct',obj.path(:,g1),obj.path(:,g2));
            x_gaze = q(1);
            y_gaze = q(2);
            %現在方位との注視点の方位差
            alpha_vcl = -theta_vcl + atan2(y_gaze-y_vcl,x_gaze-x_vcl);
            % 回転半径の計算
            r_vcl = obj.gazepoint_length/sin(alpha_vcl)/2;
            %最小回転半径の設定
            %r_vcl(abs(r_vcl)<3)=3*sign(r_vcl);
            %PurePursuitによる操舵入力
            delta = atan(obj.vehicle_length/r_vcl);
        end
    end
end

