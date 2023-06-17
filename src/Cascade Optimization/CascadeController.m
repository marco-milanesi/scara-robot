classdef CascadeController < BaseController
    %CASCADECONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        inner_controller
        outer_controller
        ref
        M
        is_ff
    end
    
    methods
        function obj = CascadeController(st,inner_controller_, outer_controller_, tau_in_, tau_out_, M_, is_ff_)
            %CASCADECONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj@BaseController(st);
            obj.inner_controller = Controller(st,inner_controller_,tau_in_);
            obj.outer_controller = Controller(st,outer_controller_,tau_out_);
            obj.ref = Queue(2);
            obj.M = System(st,M_);
            obj.is_ff = is_ff_;
        end
        
        % setta l'azione di controllo massima
        function setUMax(obj,umax_)
            % INSERIRE ASSERT SE NECESSARIO
            obj.umax=umax_;
            obj.inner_controller.setUMax(umax_(1)); 
            obj.outer_controller.setUMax(umax_(2)); 
        end

        function obj=initialize(obj)
            obj.inner_controller.initialize();
            obj.outer_controller.initialize();
            obj.M.initialize();
        end

        function obj=starting(obj,reference_,y_,u_)
            % INSERIRE ASSERT SE NECESSARIO
            obj.inner_controller.starting(y_(2),y_(2),u_);
            obj.outer_controller.starting(reference_,y_(1),0);
            obj.ref.push(reference_);
        end

        function u=computeControlAction(obj,reference_,y_)
            % INSERIRE ASSERT SE NECESSARIO
            if obj.is_ff
                ref_filt = obj.M.computeOutput(reference_);
                obj.ref.push(reference_);
                v_ff = obj.ref.differentiate(obj.st);
                v = obj.outer_controller.computeControlAction(ref_filt,y_(1));
                u = obj.inner_controller.computeControlAction(v+v_ff,y_(2));
            else
                v = obj.outer_controller.computeControlAction(reference_,y_(1));
                u = obj.inner_controller.computeControlAction(v,y_(2));
            end
        end

    end
end

