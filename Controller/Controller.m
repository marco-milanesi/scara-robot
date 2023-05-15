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
            n_par_est = length(obj.control_coef);
            time_istant_back = flip([0:obj.st:obj.st*(length(obj.error_coef)-1)]);
            obj.error.initialize( flip((e)*exp(time_istant_back/obj.tau_cl)) );
            time_istant_forw = flip(-obj.st*(n_par_est-1):obj.st:0);
            pred_error = (reference_-y_)*exp(time_istant_forw/obj.tau_cl);
            pred_control = [(u_/e+obj.deriv_u_e*obj.st*(0:n_par_est-1)).*pred_error]';
            error_total = [flip(pred_error(2:end)) obj.error.get_data()]';
            C_est=zeros(n_par_est);
            C_pred=zeros(n_par_est);
            E=zeros(n_par_est,obj.error.get_size()+n_par_est-1);
            for i = 1 : n_par_est
                C_est(i,1:end-i+1)=obj.control_coef(i:end);
                if i > 1
                    C_pred(i,1:i-1) = flip(obj.control_coef(1:i-1));
                end
                E(i,end-obj.error.get_size()+2-i:end-i+1) = -obj.error_coef;
            end
            C_pred=eye(n_par_est)-C_pred;
            obj.control.initialize(C_est\(C_pred*pred_control+E*error_total));
            error_=obj.error.get_data();
            obj.error.initialize([error_(2:end) 0]);
%             u = obj.error(1:length(obj.error_coef))*obj.error_coef+obj.control(1:length(obj.control_coef))*obj.control_coef;
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