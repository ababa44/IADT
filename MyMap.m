classdef MyMap
    %MYMAP 地図の表示用クラス
    
    properties
        objects %車両などの移動体
    end
    
    methods
        function obj = MyMap(objects)
            %MyMap このクラスのインスタンスを作成
            obj.objects = objects;
        end
        
        function plot(obj)
            % 地図内の各オブジェクトの表示
            for k = 1:numel(obj.objects)
                plot(obj.objects{k})
            end
        end
    end
end

