classdef System < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        input
        output
        input_coef
        output_coef
        st
    end

    methods
        function obj = System(st_, system_)
            if nargin < 2
                error("not enough input argument")
            end
            if ~isa(system_,'tf')
                error("controller_ must be a 'tf'")
            end
            obj.st = st_;
            sys_d=c2d(system_,st_);
            calc_coef(obj,sys_d);
            obj.input=Queue(length(obj.input_coef));
            obj.output=Queue(length(obj.output_coef));
        end

        function obj = initizlize(obj)
            obj.input.initialize();
            obj.output.initialize();
        end
        
        function y = computeOutput(obj,u_)
            
            obj.input.push(u_);
            y = obj.input(1:length(obj.input_coef))*obj.input_coef+obj.output(1:length(obj.output_coef))*obj.output_coef;
            obj.output.push(y);
        end
    end
    
    methods (Access = private)
        function calc_coef(obj, controller_)
            [num,den] = tfdata(controller_);
            obj.input_coef=(num{1}/(den{1}(1)))';
            obj.output_coef=(-den{1}(2:end)/(den{1}(1)))';
        end
    end
end