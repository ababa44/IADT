clear variables
close all

dt = 0.1;
t = 0:dt:160;

% �Q�ƋO���̐ݒ�
path_point = [
    -15,-4;
    0, 0;
    10,5;
    15,5;
    20,10;
    20,20;
    10,30;
    0,30;
    5,15;
    0,10;
    -10,25;
    -15,15;
    -20,15;
    -25,5;
    -15,-4;
    ];
% path_point = [
%     -7,-7
%     0, 0;
%     5,5;
%     10,15;
%     20,20;
%     15,40;
%     30,40;
%     50,40;
%     60,50;
%     ];

s_idx = 1:length(path_point);
d_idx = 1:0.01:length(path_point);
px_spline = interp1(s_idx, path_point(:,1), d_idx,'spline');
py_spline = interp1(s_idx, path_point(:,2), d_idx,'spline');

path = [px_spline; py_spline];

% �R���g���[���̐ݒ�
% myController = PurePursuitCarController(path);
% myController = PIDCarController(path);
myController = SimpleMPCCarController(path);
% myController = LineTraceCarController(path);

% �Ԃ̏����ݒ�
myCar = CarRectangle(0,0,0,0,0,0,1.5,3);
setController(myCar,myController);

% �\���p�n�}�̐ݒ�
myMap = MyMap({myCar});

for k = 1:numel(t)
    %�R���g���[���̌v�Z
    [vx,vy,omega,direction] = myController.calculateCarVelocity(dt);
    %���ԕ����̍X�V
    myCar.updateDirection(direction);
    %���ԑ��x�̍X�V
    myCar.updateVelocity(vx,vy,omega);
    %���Ԉʒu�̍X�V
    myCar.updatePosition(dt);
    %�L�^
    myCircleSeries(k) = copy(myCar);
    %�}��
    if mod(k,20)==1
        figure(1)
        %     cla
        hold on
        plot(myMap)
%         xlim([-5,65])
%         ylim([-5,50])
        axis equal
        grid on
        drawnow
    end
    
end

% �덷�̌v�Z
for k=1:numel(t)
    tempCircle = myCircleSeries(k);
    d(k) = tempCircle.controller.calculateShortestDistancePath(tempCircle.getPosition);
end
figure(2);
subplot(2,1,1)
plot(t,d)
xlabel('�o�ߎ���')
ylabel('�Q�ƋO������̌덷')
RMSE = sqrt(sum(d.^2)/numel(d))