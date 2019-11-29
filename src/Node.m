classdef Node
    properties
        xidx = 0;
        yidx = 0;
        yawidx = 0;
        D = 0;
        delta = 0;
        x = 0;
        y = 0;
        theta = 0;
        parent = [0,0,0];
        cost = inf;
    end
    methods
        function obj = Node(xidx,yidx,yawidx,D,delta,x,y,theta,parent,cost) % 构造函数，声明的时候就定义了
            obj.xidx = xidx;
            obj.yidx = yidx;
            obj.yawidx = yawidx;
            obj.D = D;
            obj.delta = delta;
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.parent = parent;
            obj.cost = cost;
        end
    end
end