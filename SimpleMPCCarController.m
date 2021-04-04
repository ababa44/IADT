classdef SimpleMPCCarController < CarController
    %SimpleMPCCarController 簡易的なMPCコントローラ
    %   線形化したモデルを使った簡易的なMPCをもとに自車姿勢を回転させる
    %   速度の絶対値は一定とする
    
    methods
        function obj = SimpleMPCCarController(path)
            % オブジェクトの生成
            obj@CarController(path);
        end

        function delta = calculateSteeringAngle(obj,dt)
            %MPCによる前輪転舵角指令値の計算
            
            %位置情報の取得
            car = obj.parent;
            x_vcl_ini = car.x;
            y_vcl_ini = car.y;
            theta_vcl = car.theta;

            %車両（誤差）ダイナミクスの取得
            [~,~,Fd,gd] = calculateVehicleErrorDynamics(obj,dt);
            fc = @(x,u)calculateVehicleDynamics(obj,x,u);
            %初期値の取得
            xk = [x_vcl_ini,y_vcl_ini,theta_vcl]';
            %誤差ベクトルの計算
            x0 = calculateLateralAndOrientationError(obj,xk);
            %モデルによる予測計算
            [A,B,Uref,S,W,Q,R] = obj.calculateModelPrediction(fc,xk,Fd,gd,dt,40);
            
            %最適化計算のための評価関数の係数を設定
            h = B'*Q*B+R;
            H = (h+h')/2;
            f = (A*x0+S*W)'*Q*B-Uref'*R;
            
            if license('test','optimization_toolbox')
                %Optimization Toolboxのquadprogを利用
                %最適化オプションの設定
                options = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'Display', 'off');
                %最適化計算本体
                Uopt = quadprog(H,f,[],[],[],[],[],[],[],options);
            else
                %Optimization Toolboxがない場合，標準のfminsearchで代替
                warning('No optimization Toolbox! Substituting fminsearch for quadprog.')
                %評価関数の設定
                fun = @(x)1/2*x'*H*x+f*x;
                %最適化オプションの設定
                options = optimset('Display','off');
                %最適化計算本体
                Uopt = fminsearch(fun,Uref,options);
            end
            %debug
%             xk = [x_vcl_ini,y_vcl_ini,theta_vcl]';
%             x_vcl_opt = zeros(1,N);
%             y_vcl_opt = zeros(1,N);
%             for k = 1:N
%                 uk = Uopt(k);
%                 xk = fd(xk,uk);
%                 x_vcl_opt(k) = xk(1);
%                 y_vcl_opt(k) = xk(2);
%             end
%             plot(x_vcl_opt,y_vcl_opt,'r')
%             hold on
%             axis equal

            %MPCによる操舵入力
            delta = Uopt(1);
        end
        function [A,B,U,S,W,Q,R] = calculateModelPrediction(obj,fc,xk,Fd,gd,dt,N)
            %モデルを使った予測を行う
            % X = A*x0+B*U+S*W
            
            V_vcl = obj.vehicle_speed;
            %参照経路に関する情報の取得
            %曲率
            curvature = calculateCurvature(obj);
            %参照操舵角
            delta_ref = atan(obj.vehicle_length*curvature);
            %参照方位角
            theta_ref = calculateThetaReferance(obj);
            
            %変数の次元を設定
%             xdim = 2;
            xdim = size(Fd,2);
%             xdim = length(x0);
%             udim = 1;
            udim = size(gd(0),2);
            fd = f_rk4(fc, dt);
            %変数の準備
            Qmpc = eye(xdim);
            Rmpc = eye(udim);
            A = zeros(xdim*(N+1),xdim);
            B = zeros(xdim*(N+1),udim*N);
            S = zeros(xdim*(N+1),xdim*N);
            W = zeros(N,xdim);
            Uref = zeros(N,udim);
            
            %パラメータ行列の生成
            for k = 1:N+1
                A(xdim*(k-1)+1:xdim*k,:) = Fd^(k-1);
            end
            Ap = vertcat(zeros(xdim),A);
            Ap = Ap(1:xdim*(N+1),:);
            
            %モデルを使って，Nステップ先までの予測
            for k = 1:N
                %自車から，参照経路上のどの線分に最も近いかを算出し，線分のどの位置が最短距離かを算出する
                [~,p1_idx,p2_idx] = calculateShortestDistancePath(obj,xk(1:2));
                nearest_path_vector = obj.path(:,p2_idx)-obj.path(:,p1_idx);
                relative_vcl_vector = xk(1:2)-obj.path(:,p1_idx);
                ratio = dot(nearest_path_vector,relative_vcl_vector)/norm(nearest_path_vector)^2;
                %入力として参照操舵角を利用
                uk = obj.calculateAngleDifference(theta_ref(p2_idx),theta_ref(p1_idx))*ratio+delta_ref(p1_idx);
                %状態方程式に基づいて状態を更新
                xk = fd(xk,uk);
                
                %時変パラメータの保存
                curv = (curvature(p2_idx)-curvature(p1_idx))*ratio+curvature(p1_idx);
                W(k,:) = [0,-V_vcl*curv*dt];
%                 W(k,:) = [0,-V_vcl/obj.vehicle_length*uk/cos(uk)^2*dt_mpc];
                Uref(k,1) = uk;
%                 Uref(k,1) = 0;
                B(:,k) = circshift(Ap,[xdim*(k-1),0])*gd(uk);
                S(:,xdim*(k-1)+1:xdim*k) = circshift(Ap,[xdim*(k-1),0]);
                B(1:xdim*k,k) = 0;
            end
            %計算結果の出力
            U = Uref;
            S = tril(S);
            W = W';
            W = W(:);
            Q = kron(eye(N+1),Qmpc);
            R = kron(eye(N),Rmpc);
        end

        function [F,g,Fd,gd] = calculateVehicleErrorDynamics(obj,dt)
            %車両の誤差モデル
            L = obj.vehicle_length;
            V = obj.vehicle_speed;

            F = [0,V;
                0,0];
            g = @(x)[0,V/L]';
            
            %オイラー法による離散化
            Fd = eye(size(F))+F*dt;
            gd = @(delta_ref)g(delta_ref)*dt;
        end
        function curvature = calculateCurvature(obj)
            %参照軌道の曲率の計算
            path = obj.path;
            curvature = zeros(1,length(path));
            
            %隣接する3点(p1,p2,p3)で構成される三角形の外接円の半径の逆数を曲率として計算する
            for i = 2:length(path)-1
                %p1-p2の線分
                v12 = path(:,i)-path(:,i-1);
                %p2-p3の線分
                v23 = path(:,i+1)-path(:,i);
                %p3-p1の線分
                v31 = path(:,i-1)-path(:,i+1);
                
                %p3の位置が直線p1p2の右側ならば右回転(-)，左側ならば左回転(+)
                %外積計算のために次元を一つ追加
                v3 = -[v31;0];
                v2 = [v12;0];
                %外積計算で左右回転を判定
                nz = [0,0,1]*cross(v2,v3);
                
                %以下の２つから曲率を計算
                %1.ヘロンの公式による面積の計算S=sqrt((a+b+c)(-a+b+c)(a-b+c)(a+b-c))/4
                %2.外接円の半径と三角形の面積の関係1/R=4S/abc
                a = norm(v12);
                b = norm(v23);
                c = norm(v31);
                v = [a,b,c]';
                M = [1,1,1;-1,1,1;1,-1,1;1,1,-1];
                curvature(i) = sign(nz)*sqrt(prod(M*v))/(a*b*c);
            end
        end
        function theta_ref = calculateThetaReferance(obj)
            %参照軌道の方位を計算
            path = obj.path;
            %参照軌道のベクトルから方位を計算
            theta_ref = atan2(diff(path(2,:)),diff(path(1,:)));
            %差分を取るとデータ点が一つ減るので，最後の値でパディング
            theta_ref = [theta_ref,theta_ref(end)];
        end
    end
end

