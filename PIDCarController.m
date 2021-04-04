classdef PIDCarController < CarController
    %PIDCarController PIDコントローラ
    %   PIDをもとに横偏差と方位偏差を0にする方向に自車姿勢を回転させる
    %   速度の絶対値は一定とする
    
    properties
        KP = [1.5,1.5]; % Pゲイン
        KD = [0.1,0.1]; % Dゲイン
        KI = [0.7,0.7]; % Iゲイン
        integral_e_v = [0;0]; %誤差の積分値
        previous_e_v = [0;0]; %誤差の前回値（微分用）
    end
    
    methods
        function obj = PIDCarController(path)
            % オブジェクトの生成
            obj@CarController(path);
        end
        
        function delta = calculateSteeringAngle(obj,dt)
            %PIDによる前輪転舵角指令値の計算
            car = obj.parent;
            x_vcl = car.x;
            y_vcl = car.y;
            theta_vcl = car.theta;
            xk = [x_vcl,y_vcl,theta_vcl]';

            %誤差ベクトル
            e_v = calculateLateralAndOrientationError(obj,xk);
            %誤差ベクトルの積分
            obj.integral_e_v = obj.integral_e_v+e_v*dt;
            %誤差ベクトルの微分
            de_v = (e_v-obj.previous_e_v)/dt;
            obj.previous_e_v = e_v;
            %PIDによる操舵入力
            delta = -(obj.KP*e_v+obj.KI*obj.integral_e_v+obj.KD*de_v);
        end
    end
end

