classdef Controller < BaseController
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        error
        control
        error_coef
        control_coef
        u_max
        u_min
    end

    methods
        function obj = Controller(st,controller_, u_sat_, size_)
            if ~isa(controller_,'tf') || controller_.TS<=0
                error("controller_ must be a 'tf' and it has to be a sampled system (controller_.Ts > 0)")
            end
            obj@BaseController(st);
            calc_coef(obj,controller_);
            if nargin < 3
                u_sat_ = [0 1];
            elseif length(u_sat_) ~= 2
                error("The length of the maximum output acceptable must be 2 [u_min u_max]")
            elseif u_sat_(1) > u_sat_(2)
                error("The saturation of the actuator must be passed as a vector like '[u_min u_max]'")
            end
            if nargin < 4
                size_=max([length(obj.error_coef) length(obj.control_coef)]);
            elseif ~isa(size_,'double')
                error("size_ has to be a double")
            end
            obj.error=Queue(size_);
            obj.control=Queue(size_);
            obj.u_min=u_sat_(1);
            obj.u_max=u_sat_(2);
        end
        
        function u_sat = get_saturation(obj)
            u_sat = [obj.u_min obj.u_max];
        end
        
        function size = get_size(obj)
            size = obj.control.get_size();
        end

        function obj = initialize(obj)
            obj.error.initialize();
            obj.control.initialize();
        end
        
        function obj = starting(obj,error_,control_)
            if nargin < 3
                error("not enought input argument")
            end
            if length(error_) ~= obj.error.get_size && length(control_) ~= obj.control.get_size
                error("initial values for error_ and for control_ have the wrong size it should be "+obj.error.get_size)
            end
            obj.error.initialize(error_);
            obj.control.initialize(control_);
        end
        
        function u = computeControlAction(obj,reference_,y_)
            e = reference_ - y_;
            obj.error.push(e);
            u = obj.error(1:length(obj.error_coef))*obj.error_coef+obj.control(1:length(obj.control_coef))*obj.control_coef;
            if u > obj.u_max
                u = obj.u_max;
            elseif u < obj.u_min
                u = obj.u_min;
            end
            obj.control.push(u);
        end

        function [error, control] = stop(obj)
            error = obj.error.get_data();
            control = obj.control.get_data();
        end
    end
    
    methods (Access = private)
        function calc_coef(obj, controller_)
            [num,den] = tfdata(controller_);
            obj.error_coef=(num{1}/(den{1}(1)))';
            obj.control_coef=(-den{1}(2:end)/(den{1}(1)))';
        end
    end
end