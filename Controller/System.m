classdef System < BaseController
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        input
        output
        input_coef
        output_coef
        u_max
        u_min
    end

    methods
        function obj = System(st, system_)
            if ~isa(system_,'tf') || system_.TS<=0
                error("controller_ must be a 'tf' and it has to be a sampled system (controller_.Ts > 0)")
            end
            obj@BaseController(st);
            calc_coef(obj,system_);
            obj.input=Queue(length(obj.input_coef));
            obj.output=Queue(length(obj.output_coef));
        end

        function obj = initizlize(obj)
            obj.input.initialize();
            obj.output.initialize();
        end
        
        function u = computeControlAction(obj,reference_,y_)
            e = reference_ - y_;
            obj.error.push(e);
            u = obj.error(1:length(obj.error_coef))*obj.error_coef+obj.control(1:length(obj.output_coef))*obj.output_coef;
            obj.control.push(u);
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