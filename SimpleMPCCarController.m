classdef SimpleMPCCarController < CarController
    %SimpleMPCCarController �ȈՓI��MPC�R���g���[��
    %   ���`���������f�����g�����ȈՓI��MPC�����ƂɎ��Ԏp������]������
    %   ���x�̐�Βl�͈��Ƃ���
    
    methods
        function obj = SimpleMPCCarController(path)
            % �I�u�W�F�N�g�̐���
            obj@CarController(path);
        end

        function delta = calculateSteeringAngle(obj,dt)
            %MPC�ɂ��O�֓]�Ǌp�w�ߒl�̌v�Z
            
            %�ʒu���̎擾
            car = obj.parent;
            x_vcl_ini = car.x;
            y_vcl_ini = car.y;
            theta_vcl = car.theta;

            %�ԗ��i�덷�j�_�C�i�~�N�X�̎擾
            [~,~,Fd,gd] = calculateVehicleErrorDynamics(obj,dt);
            fc = @(x,u)calculateVehicleDynamics(obj,x,u);
            %�����l�̎擾
            xk = [x_vcl_ini,y_vcl_ini,theta_vcl]';
            %�덷�x�N�g���̌v�Z
            x0 = calculateLateralAndOrientationError(obj,xk);
            %���f���ɂ��\���v�Z
            [A,B,Uref,S,W,Q,R] = obj.calculateModelPrediction(fc,xk,Fd,gd,dt,40);
            
            %�œK���v�Z�̂��߂̕]���֐��̌W����ݒ�
            h = B'*Q*B+R;
            H = (h+h')/2;
            f = (A*x0+S*W)'*Q*B-Uref'*R;
            
            if license('test','optimization_toolbox')
                %Optimization Toolbox��quadprog�𗘗p
                %�œK���I�v�V�����̐ݒ�
                options = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'Display', 'off');
                %�œK���v�Z�{��
                Uopt = quadprog(H,f,[],[],[],[],[],[],[],options);
            else
                %Optimization Toolbox���Ȃ��ꍇ�C�W����fminsearch�ő��
                warning('No optimization Toolbox! Substituting fminsearch for quadprog.')
                %�]���֐��̐ݒ�
                fun = @(x)1/2*x'*H*x+f*x;
                %�œK���I�v�V�����̐ݒ�
                options = optimset('Display','off');
                %�œK���v�Z�{��
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

            %MPC�ɂ�鑀�Ǔ���
            delta = Uopt(1);
        end
        function [A,B,U,S,W,Q,R] = calculateModelPrediction(obj,fc,xk,Fd,gd,dt,N)
            %���f�����g�����\�����s��
            % X = A*x0+B*U+S*W
            
            V_vcl = obj.vehicle_speed;
            %�Q�ƌo�H�Ɋւ�����̎擾
            %�ȗ�
            curvature = calculateCurvature(obj);
            %�Q�Ƒ��Ǌp
            delta_ref = atan(obj.vehicle_length*curvature);
            %�Q�ƕ��ʊp
            theta_ref = calculateThetaReferance(obj);
            
            %�ϐ��̎�����ݒ�
%             xdim = 2;
            xdim = size(Fd,2);
%             xdim = length(x0);
%             udim = 1;
            udim = size(gd(0),2);
            fd = f_rk4(fc, dt);
            %�ϐ��̏���
            Qmpc = eye(xdim);
            Rmpc = eye(udim);
            A = zeros(xdim*(N+1),xdim);
            B = zeros(xdim*(N+1),udim*N);
            S = zeros(xdim*(N+1),xdim*N);
            W = zeros(N,xdim);
            Uref = zeros(N,udim);
            
            %�p�����[�^�s��̐���
            for k = 1:N+1
                A(xdim*(k-1)+1:xdim*k,:) = Fd^(k-1);
            end
            Ap = vertcat(zeros(xdim),A);
            Ap = Ap(1:xdim*(N+1),:);
            
            %���f�����g���āCN�X�e�b�v��܂ł̗\��
            for k = 1:N
                %���Ԃ���C�Q�ƌo�H��̂ǂ̐����ɍł��߂������Z�o���C�����̂ǂ̈ʒu���ŒZ���������Z�o����
                [~,p1_idx,p2_idx] = calculateShortestDistancePath(obj,xk(1:2));
                nearest_path_vector = obj.path(:,p2_idx)-obj.path(:,p1_idx);
                relative_vcl_vector = xk(1:2)-obj.path(:,p1_idx);
                ratio = dot(nearest_path_vector,relative_vcl_vector)/norm(nearest_path_vector)^2;
                %���͂Ƃ��ĎQ�Ƒ��Ǌp�𗘗p
                uk = obj.calculateAngleDifference(theta_ref(p2_idx),theta_ref(p1_idx))*ratio+delta_ref(p1_idx);
                %��ԕ������Ɋ�Â��ď�Ԃ��X�V
                xk = fd(xk,uk);
                
                %���σp�����[�^�̕ۑ�
                curv = (curvature(p2_idx)-curvature(p1_idx))*ratio+curvature(p1_idx);
                W(k,:) = [0,-V_vcl*curv*dt];
%                 W(k,:) = [0,-V_vcl/obj.vehicle_length*uk/cos(uk)^2*dt_mpc];
                Uref(k,1) = uk;
%                 Uref(k,1) = 0;
                B(:,k) = circshift(Ap,[xdim*(k-1),0])*gd(uk);
                S(:,xdim*(k-1)+1:xdim*k) = circshift(Ap,[xdim*(k-1),0]);
                B(1:xdim*k,k) = 0;
            end
            %�v�Z���ʂ̏o��
            U = Uref;
            S = tril(S);
            W = W';
            W = W(:);
            Q = kron(eye(N+1),Qmpc);
            R = kron(eye(N),Rmpc);
        end

        function [F,g,Fd,gd] = calculateVehicleErrorDynamics(obj,dt)
            %�ԗ��̌덷���f��
            L = obj.vehicle_length;
            V = obj.vehicle_speed;

            F = [0,V;
                0,0];
            g = @(x)[0,V/L]';
            
            %�I�C���[�@�ɂ�闣�U��
            Fd = eye(size(F))+F*dt;
            gd = @(delta_ref)g(delta_ref)*dt;
        end
        function curvature = calculateCurvature(obj)
            %�Q�ƋO���̋ȗ��̌v�Z
            path = obj.path;
            curvature = zeros(1,length(path));
            
            %�אڂ���3�_(p1,p2,p3)�ō\�������O�p�`�̊O�ډ~�̔��a�̋t�����ȗ��Ƃ��Čv�Z����
            for i = 2:length(path)-1
                %p1-p2�̐���
                v12 = path(:,i)-path(:,i-1);
                %p2-p3�̐���
                v23 = path(:,i+1)-path(:,i);
                %p3-p1�̐���
                v31 = path(:,i-1)-path(:,i+1);
                
                %p3�̈ʒu������p1p2�̉E���Ȃ�ΉE��](-)�C�����Ȃ�΍���](+)
                %�O�όv�Z�̂��߂Ɏ�������ǉ�
                v3 = -[v31;0];
                v2 = [v12;0];
                %�O�όv�Z�ō��E��]�𔻒�
                nz = [0,0,1]*cross(v2,v3);
                
                %�ȉ��̂Q����ȗ����v�Z
                %1.�w�����̌����ɂ��ʐς̌v�ZS=sqrt((a+b+c)(-a+b+c)(a-b+c)(a+b-c))/4
                %2.�O�ډ~�̔��a�ƎO�p�`�̖ʐς̊֌W1/R=4S/abc
                a = norm(v12);
                b = norm(v23);
                c = norm(v31);
                v = [a,b,c]';
                M = [1,1,1;-1,1,1;1,-1,1;1,1,-1];
                curvature(i) = sign(nz)*sqrt(prod(M*v))/(a*b*c);
            end
        end
        function theta_ref = calculateThetaReferance(obj)
            %�Q�ƋO���̕��ʂ��v�Z
            path = obj.path;
            %�Q�ƋO���̃x�N�g��������ʂ��v�Z
            theta_ref = atan2(diff(path(2,:)),diff(path(1,:)));
            %���������ƃf�[�^�_�������̂ŁC�Ō�̒l�Ńp�f�B���O
            theta_ref = [theta_ref,theta_ref(end)];
        end
    end
end

