classdef MyRectangle < matlab.mixin.Copyable
    %RECTANGLE 長方形の表示用クラス
    %   xy平面上での長方形の表示
    
    properties
        x % 中心のx座標
        y % 中心のy座標
        w % x軸方向幅
        h % y軸方向高さ
        theta % 向き
    end
    
    methods
        function obj = MyRectangle(x, y, width, height, theta)
            % オブジェクトの生成
            obj.x = x;
            obj.y = y;
            obj.w = width;
            obj.h = height;
            obj.theta = theta;
        end
        function p = getPosition(obj)
            % 中心座標をベクトルで取得
            p = [obj.x; obj.y];
        end
        function plot(obj,varargin)
            % 長方形の表示
            
            % 長方形の位置・サイズを取得
            x0 = obj.x;
            y0 = obj.y;
            w0 = obj.w;
            h0 = obj.h;
            % 原点中心の長方形をBounding Box形式で指定
            pos = [-w0/2, -h0/2, w0, h0];
            
            % 並進行列（原点->車両中心）
            T = makehgtform('translate',[x0,y0,0]);
            % 回転行列（x軸方向を0，反時計回りの角度で表した車両の向き）
            Rz = makehgtform('zrotate',obj.theta);
            
            % 座標変換オブジェクトの作成
            t = hgtransform;
            % 上記を親にして，原点中心の長方形を作成（表示）
            rectangle('Position', pos, 'Parent', t, varargin{:});
            % 車両中心座標・車両の向きをもとに回転・並進
            set(t,'Matrix',T*Rz)
        end
    end
    
end