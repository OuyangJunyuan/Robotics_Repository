function [varargout] = myconv(varargin)
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if nargin~=0
    ans2=varargin{1};
    if nargin>1
        for i=2:nargin
            ans1=conv(varargin{i},ans2);
            ans2=ans1;
        end  
    end
    varargout={ans2};
end
end

