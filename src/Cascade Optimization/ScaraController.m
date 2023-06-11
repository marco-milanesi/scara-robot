classdef ScaraController < BaseController 
    % Implementazione di PI per i due giunti
    properties  (Access = protected)
        joint1_controller
        joint2_controller
    end
    methods
        function obj=ScaraController(st,j1_inner_controller_, j1_outer_controller_, j1_tau_in_, j1_tau_out_, M1_, j2_inner_controller_, j2_outer_controller_, j2_tau_in_, j2_tau_out_, M2_)
            % INSERIRE ASSERT SE NECESSARIO
            
            obj@BaseController(st);
            obj.joint1_controller=CascadeController(st,j1_inner_controller_, j1_outer_controller_, j1_tau_in_, j1_tau_out_, M1_);
            obj.joint2_controller=CascadeController(st,j2_inner_controller_, j2_outer_controller_, j2_tau_in_, j2_tau_out_, M2_);
        end

        % setta l'azione di controllo massima
        function setUMax(obj,umax)
            % INSERIRE ASSERT SE NECESSARIO
            obj.umax=umax;
            obj.joint1_controller.setUMax([umax(1) inf]); 
            obj.joint2_controller.setUMax([umax(2) inf]); 
        end
        
        function obj=initialize(obj)
            obj.joint1_controller.initialize();
            obj.joint2_controller.initialize();
        end

        function obj=starting(obj,reference,y,u)
            % INSERIRE ASSERT SE NECESSARIO
            obj.joint1_controller.starting(reference(1),[y(1) y(3)],u(1));
            obj.joint1_controller.starting(reference(2),[y(2) y(4)],u(2));
        end

        function u=computeControlAction(obj,reference,y)
            % INSERIRE ASSERT SE NECESSARIO
            u(1,1)=obj.joint1_controller.computeControlAction(reference(1),[y(1) y(3)]);
            u(2,1)=obj.joint2_controller.computeControlAction(reference(2),[y(2) y(4)]);
        end
    end
end