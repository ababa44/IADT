clear variables
close all

dt = 0.1;
t = 0:dt:160;

% 参照軌道の設定
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

% コントローラの設定
% myController = PurePursuitCarController(path);
% myController = PIDCarController(path);
myController = SimpleMPCCarController(path);
% myController = LineTraceCarController(path);

% 車の初期設定
myCar = CarRectangle(0,0,0,0,0,0,1.5,3);
setController(myCar,myController);

% 表示用地図の設定
myMap = MyMap({myCar});

for k = 1:numel(t)
    %コントローラの計算
    [vx,vy,omega,direction] = myController.calculateCarVelocity(dt);
    %自車方向の更新
    myCar.updateDirection(direction);
    %自車速度の更新
    myCar.updateVelocity(vx,vy,omega);
    %自車位置の更新
    myCar.updatePosition(dt);
    %記録
    myCircleSeries(k) = copy(myCar);
    %図示
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

% 誤差の計算
for k=1:numel(t)
    tempCircle = myCircleSeries(k);
    d(k) = tempCircle.controller.calculateShortestDistancePath(tempCircle.getPosition);
end
figure(2);
subplot(2,1,1)
plot(t,d)
xlabel('経過時間')
ylabel('参照軌道からの誤差')
RMSE = sqrt(sum(d.^2)/numel(d))