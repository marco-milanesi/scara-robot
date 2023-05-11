classdef Controller < BaseController
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        error
        control
        error_coef
        control_coef
        tau_cl
        deriv_u_e
        dc_gain
    end

    methods
        function obj = Controller(st,controller_,tau_cl_)
            if nargin < 3
                error('Not enough input')
            end
            if ~isa(controller_,'tf')
                error("controller_ must be a 'tf'.")
            end
            obj@BaseController(st);
            s=tf('s');
            obj.deriv_u_e = dcgain(controller_*s);
            if obj.deriv_u_e == 0
                obj.dc_gain = dcgain(controller_);
            end
            controller_d=c2d(controller_,st);
            calc_coef(obj,controller_d);
            if nargin < 3
                u_sat_ = 1;
            end
            obj.error=Queue(length(obj.error_coef));
            obj.control=Queue(length(obj.control_coef));
            obj.tau_cl=tau_cl_;
        end
        
        function u_sat = get_saturation(obj)
            u_sat = obj.umax;
        end

        function obj = initialize(obj)
            obj.error.initialize();
            obj.control.initialize();
        end
        
        function obj=starting(obj,reference_,y_,u_)
            if nargin < 4
                error("not enought input argument")
            end
            e = reference_-y_;
            n_par_est = length(obj.control_coef)-1;
            obj.error.initialize((e)*exp([-obj.st*length(obj.error_coef):obj.st:0]/obj.tau_cl));
            pred_error = (reference_-y_)*exp([0:st:obj.st*n_par_est]/obj.tau_cl);
            pred_control = (u_/e+obj.deriv_u_e*obj.st*[1:n_par_est]).*pred_error;

        end
        
        function u = computeControlAction(obj,reference_,y_)
            e = reference_ - y_;
            obj.error.push(e);
            u = obj.error(1:length(obj.error_coef))*obj.error_coef+obj.control(1:length(obj.control_coef))*obj.control_coef;
            if u > obj.umax
                u = obj.umax;
            elseif u < -obj.umax
                u = -obj.umax;
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