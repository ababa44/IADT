classdef CarController < matlab.mixin.Copyable
    %CARCONTROLLER 車両コントローラ用のベースクラス
    %   各コントローラに共通で必要な機能の実装
    %   ハンドルクラスとして実装し，コピーを可能とする．
    
    properties
        path %参照軌道
        parent %親（制御対象の車両）
        
        vehicle_speed = 1; %車速（進行方向に対する速度）
        vehicle_length = 2; %車両の前後輪間の長さ
    end
    
    methods
        function obj = CarController(path)
            % オブジェクトの生成
            obj.path = path;
        end
        
        function [vx,vy,omega,direction] = calculateCarVelocity(obj,dt)
            % 速度・角速度・車両進行方向の指令値の計算
            
            car = obj.parent;
            x_vcl = car.x;
            y_vcl = car.y;
            theta_vcl = car.theta;
            %前輪転舵角の指令値の計算
            delta = calculateSteeringAngle(obj,dt);

            %車両モデルを用いた速度・角速度の計算
            xk = [x_vcl,y_vcl,theta_vcl]';
            xout = calculateVehicleDynamics(obj,xk,delta);
            %出力
            vx = xout(1);
            vy = xout(2);
            omega = xout(3);
            direction = 'Forward';%とりあえず前進のみ
        end
        function delta = calculateSteeringAngle(obj,dt)
            % 前輪転舵角の指令値の計算（抽象メソッド）
            delta = 0;
        end
        function x = calculateVehicleDynamics(obj,x,u)
            % 車両モデル
            L = obj.vehicle_length;
            V = obj.vehicle_speed;
            theta = x(3);
            delta = u;
            
            x(1) = V*cos(theta);
            x(2) = V*sin(theta);
            x(3) = V*tan(delta)/L;
        end
        function [distance,index_point1,index_point2] = calculateShortestDistancePath(obj,point)
            % 現在位置から最も近いパス上の線分を算出する
            % そのときの距離と，パスの何番目の点と点を結んだ線分か算出
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
            %参照軌道に対する現在位置・方位の誤差を算出する
            
            %現在位置から最も近いパス上の線分を算出する（同時に横偏差を算出）
            [lateral_error,p1_idx,p2_idx] = calculateShortestDistancePath(obj,xk(1:2));
            %最も近いパス上の線分ベクトル
            nearest_path_vector = obj.path(:,p2_idx)-obj.path(:,p1_idx);
            %線分と自車の位置関係を調べる（外積計算）
            %線分の始点から自車までのベクトルを用意
            relative_vcl_vector = xk(1:2)-obj.path(:,p1_idx);
            %線分ベクトルとの外積を計算
            Z = cross([nearest_path_vector;0],[relative_vcl_vector;0]);
            %z要素の符号により左右が決まるので，横偏差に符号をつける
            lateral_error = sign(Z(3))*lateral_error;
            %線分ベクトルの方向
            theta_path = atan2(nearest_path_vector(2),nearest_path_vector(1));
            %方位偏差
            theta_error = obj.calculateAngleDifference(xk(3),theta_path);

            %誤差ベクトル
            error_vector = [lateral_error;theta_error];
        end
        function plot(obj,varargin)
            %コントローラに設定された参照軌道の図示
            line(obj.path(1,:),obj.path(2,:),varargin{:})
        end
    end
    methods (Static)
        function angle_diff = calculateAngleDifference(theta_target,theta_base)
            %角度偏差を出すために複素数で処理
            complex_target = cos(theta_target)+1i*sin(theta_target);
            complex_base = cos(theta_base)+1i*sin(theta_base);
            %角度偏差
            complex_error = complex_target/complex_base;
            angle_diff = angle(complex_error);
        end
        function [ distance, nearest_point ] = calculatePointLinesegmentDistance( point, end_point1, end_point2 )
            %点と線分の距離を計算（同時にその時の線分上の最も近い点を計算）
            
            % 内積を使って，点pの線分に対する位置関係を特定する
            t = dot(point-end_point1, end_point2-end_point1)/sum((end_point1-end_point2).^2);
            if t < 0
                % 点pの位置が，線分の端点p1よりも外側の場合
                nearest_point = end_point1;
            elseif t > 1
                % 点pの位置が，線分の端点p2よりも外側の場合
                nearest_point = end_point2;
            else
                % 点pの位置が，線分の内側の場合
                nearest_point = end_point1+t*(end_point2-end_point1);
            end
            distance = norm(point-nearest_point);
        end
    end
end

