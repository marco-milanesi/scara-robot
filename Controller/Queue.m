classdef Queue < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        data
        size
    end

    methods
        function obj = Queue(size_)
            obj.data = zeros(1,size_);
            obj.size = size_;
        end
        
        function data = get_data(obj)
            data = obj.data;
        end

        function size = get_size(obj)
            size = obj.size;
        end

        function obj_copy = push(obj,element_)
            obj.data=circshift(obj.data,1);
            obj.data(1)=element_;
            obj_copy=obj;
        end

        function dot_prd = mtimes(obj, vec_)
            if isa(vec_,'double')
                if length(vec_) ~= obj.size
                    error("The length of the input vector must be equal to size")
                end
                dot_prd = obj.data*vec_;
            elseif isa(vec_,'Queue')
                dot_prd = obj.data*obj_.get_data();
            else
                error("you are trying to make a dot product witha a "+class(vec_)+" that is not supported. The only supported types are: 'double' and 'Queue'")
            end
        end

        function varargout = subsref(obj,subs)
            switch subs(1).type
                case '()' 
                    value=obj.data(subs.subs{1});
                    varargout={builtin('subsref',value,subs)};
                otherwise   
                    [varargout{1:nargout}] = builtin('subsref',obj,subs);
            end
        end

        function initialize(obj,data_)
            if nargin < 2
                obj.data=zeros(1,obj.size);
            else
                if length(data_) ~= obj.size && ~isscalar(data_)
                    error("the length of the initial data_ is incorect")
                elseif isscalar(data_)
                    obj.data = data_*ones(obj.size,1);
                elseif size(data_,1) ~= 1
                    obj.data = data_';
                else
                    obj.data = data_;
                end
            end
        end

    end
end