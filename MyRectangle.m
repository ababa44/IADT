classdef MyRectangle < matlab.mixin.Copyable
    %RECTANGLE �����`�̕\���p�N���X
    %   xy���ʏ�ł̒����`�̕\��
    
    properties
        x % ���S��x���W
        y % ���S��y���W
        w % x��������
        h % y����������
        theta % ����
    end
    
    methods
        function obj = MyRectangle(x, y, width, height, theta)
            % �I�u�W�F�N�g�̐���
            obj.x = x;
            obj.y = y;
            obj.w = width;
            obj.h = height;
            obj.theta = theta;
        end
        function p = getPosition(obj)
            % ���S���W���x�N�g���Ŏ擾
            p = [obj.x; obj.y];
        end
        function plot(obj,varargin)
            % �����`�̕\��
            
            % �����`�̈ʒu�E�T�C�Y���擾
            x0 = obj.x;
            y0 = obj.y;
            w0 = obj.w;
            h0 = obj.h;
            % ���_���S�̒����`��Bounding Box�`���Ŏw��
            pos = [-w0/2, -h0/2, w0, h0];
            
            % ���i�s��i���_->�ԗ����S�j
            T = makehgtform('translate',[x0,y0,0]);
            % ��]�s��ix��������0�C�����v���̊p�x�ŕ\�����ԗ��̌����j
            Rz = makehgtform('zrotate',obj.theta);
            
            % ���W�ϊ��I�u�W�F�N�g�̍쐬
            t = hgtransform;
            % ��L��e�ɂ��āC���_���S�̒����`���쐬�i�\���j
            rectangle('Position', pos, 'Parent', t, varargin{:});
            % �ԗ����S���W�E�ԗ��̌��������Ƃɉ�]�E���i
            set(t,'Matrix',T*Rz)
        end
    end
    
end