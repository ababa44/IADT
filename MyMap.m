classdef MyMap
    %MYMAP �n�}�̕\���p�N���X
    
    properties
        objects %�ԗ��Ȃǂ̈ړ���
    end
    
    methods
        function obj = MyMap(objects)
            %MyMap ���̃N���X�̃C���X�^���X���쐬
            obj.objects = objects;
        end
        
        function plot(obj)
            % �n�}���̊e�I�u�W�F�N�g�̕\��
            for k = 1:numel(obj.objects)
                plot(obj.objects{k})
            end
        end
    end
end

