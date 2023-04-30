classdef ElasticTransmission < MechanicalSystem
    % implement an elastic transmission sistema
    properties  (Access = protected)
        A
        B
        C
        static_friction
    end

    methods  (Access = public)
        function obj=ElasticTransmission(st)
            obj@MechanicalSystem(st); % chiama il costruttore del padre
            obj.input_names={'motor torque'};
            obj.output_names={'motor position','motor velocity'};
       
            % Jm motor inertia 
            % Jl link inertia
            % k spring stiffness
            % c spring damping
            % cm motor viscuous friction
            % static_friction 
            %
            % x=[motor_pos, link_pos, motor_vel, link_vel]
            %
            % u=motor_eff
            %
            % Jm*Derivative(motor_vel) = u - cm*motor_vel
            % -k*(motor_pos-link_pos) - c*(motor_vel-link_vel) -
            % static_friction*sign(motor_vel)
            %
            % Jl*Derivative(link_vel)  = k*(motor_pos-link_pos) + c*(motor_vel-link_vel)

            obj.x=zeros(4,1);
            obj.x0=zeros(4,1);

            Jm=2e-2;
            Jc=4e-2;

            k=1000;
            c=.2;
            cm=.01;

            obj.static_friction=0.3;

            obj.A=[0 0 1 0;
                0 0 0 1;
                -k/Jm k/Jm -c/Jm-cm/Jm c/Jm;
                k/Jc -k/Jc c/Jc -c/Jc];
            obj.B=[0;
                0;
                1/Jm;
                0];
            obj.C=[1 0 0 0;
                0 0 1 0];

            obj.order=size(obj.A,2);
            obj.num_input=size(obj.B,2);
            obj.num_output=size(obj.C,1);
            obj.sigma_y=[1e-6;1e-3];
        end
    end

    methods  (Access = protected)
        function Dx=stateFunction(obj,x,u,t)
            Dx=obj.A*x+obj.B*(u-obj.static_friction*tanh(obj.x(3)*30));
        end
        function y=outputFunction(obj)
            y=obj.C*obj.x+obj.sigma_y.*randn(length(obj.sigma_y),1);
        end
    end
end