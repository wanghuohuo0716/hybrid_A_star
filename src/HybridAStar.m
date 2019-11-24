function [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure)  
    veh = Vehicle;
    cfg = Configure;
    mres = cfg.MOTION_RESOLUTION; % motino resolution 
    % use grid a star search result as heuristic cost
    [isok,xidx,yidx,thidx] = CalcIdx(Start(1),Start(2),Start(3),cfg);
    if isok
        tnode = Node(xidx,yidx,thidx,mres,0,Start(1),Start(2),Start(3),[xidx,yidx,thidx],0);
    end
    Open = [tnode];
    Close = [];
%     [isok,xidx,yidx,thidx] = CalcIdx(End(1),End(2),End(3),cfg);
%     if isok
%         goal = Node(xidx,yidx,thidx,0,0,End,inf);
%     end
    x = [];
    y = [];
    th = [];
    D = [];
    delta = [];
    while ~isempty(Open)
        % pop the least cost node from open to close
        [wknode,Open] = PopNode(Open,cfg);
        [isok,idx] = inNodes(wknode,Close);
        if isok
            Close(idx) = wknode;
        else
            Close = [Close, wknode];
        end
        [isok,path] = AnalysticExpantion([wknode.x,wknode.y,wknode.theta],End,veh,cfg);
        if  isok
            %put wknode to the end of close
            Close(end+1) = wknode;
            Close(idx) = [];
            [x,y,th,D,delta] = getFinalPath(path,Close,veh,cfg);
            break
        end
        [Open,Close] = Update(wknode,Open,Close,veh,cfg);
    end   
%     [isok,path] = AnalysticExpantion(Start,End,Vehicle,Configure);
end

function [x,y,th,D,delta] = getFinalPath(path,Close,veh,cfg)
    wknode = Close(end);
    Close(end) = [];
    nodes = [wknode];
    while ~isempty(Close)
        n = length(Close);
        parent = wknode.parent;
        for i = n:-1:1
            flag = 0;
            tnode = Close(i);
            if tnode.xidx == parent(1)...
                    && tnode.yidx == parent(2)...
                    && tnode.yawidx == parent(3)
                nodes(end+1) = tnode;
                wknode = tnode;
                flag = 1;
                break
            end
        end
        Close(i) = [];        
    end
    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    gres = cfg.XY_GRID_RESOLUTION;
    % decrease one step, caz node origin is consider in
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1;
    x = [];
    y = [];
    th = [];
    D = [];
    delta = [];
    flag = 0;
    if length(nodes) >= 2
        % caz first node is start point, we ignore this node
        for i = length(nodes):-1:2
            tnode = nodes(i);
            ttnode = nodes(i-1);
            % initial point
            px = tnode.x;
            py = tnode.y;
            pth = tnode.theta;
            x = [x, px];
            y = [y, py];
            th = [th, pth];
            D = [D, ttnode.D];
            delta = [delta, ttnode.delta];
            for idx = 1:nlist
                [px,py,pth] = VehicleDynamic(px,py,pth,ttnode.D,ttnode.delta,veh.WB);
                x = [x, px];
                y = [y, py];
                th = [th, pth];
                D = [D, ttnode.D];
                delta = [delta, ttnode.delta];
            end
            % delete last point
            if i ~= 2
                x(end) = [];
                y(end) = [];
                th(end) = [];
                D(end) = [];
                delta(end) = [];
            end
        end
    else
        % if just rs path so we will get path without node
        flag = 1;
        tnode = nodes(1);
        px = tnode.x;
        py = tnode.y;
        pth = tnode.theta;
        x = [x, px];
        y = [y, py];
        th = [th, pth];
    end
    types = path.type;
    t = rmin*path.t;
    u = rmin*path.u;
    v = rmin*path.v;
    w = rmin*path.w;
    segs = [t,u,v,w,rmin*path.x;];% avoid duplicate of x
    for i = 1:5
        if segs(i) == 0
            continue
        end
        s = sign(segs(i));
        
        if types(i) == 'S'          
            tdelta = 0;
        elseif types(i) == 'L'
            tdelta = smax;
        elseif types(i) == 'R'
            tdelta = -smax;
        else
            % do nothing
        end
        if flag == 1
            % initialization if only rs path
            D = [D, s*mres];
            delta = [delta, tdelta];
            flag = 0;
        end
        for idx = 1:round(abs(segs(i))/mres)                
           	[px,py,pth] = VehicleDynamic(px,py,pth,s*mres,tdelta,veh.WB);
            x = [x, px];
            y = [y, py];
            th = [th, pth];
            D = [D, s*mres];
            delta = [delta, tdelta];         
        end
    end
end

function [Open,Close] = Update(wknode,Open,Close,veh,cfg)
    mres = cfg.MOTION_RESOLUTION; % motino resolution    
    smax = veh.MAX_STEER; % maximum steering angle
    sres = smax/cfg.N_STEER; % steering resolution  
    % all possible control input
    for D = [-mres,mres]
        for delta = [-smax:sres:-sres,0,sres:sres:smax]
            [isok,tnode] = CalcNextNode(wknode,D,delta,veh,cfg);
            if isok == false
                continue
            end
            % if already in close, skip it
            [isok,~] = inNodes(tnode,Close);
            if isok
                continue
            end 
            [isok,idx] = inNodes(tnode,Open);
            if isok
                % in same grid but have different cost
                tcost = TotalCost(tnode,cfg);
                ttnode = Open(idx);
                ttcost = TotalCost(ttnode,cfg);
                if tcost < ttcost
                    Open(idx) = tnode;
                end
            else
                Open(end+1) = tnode;
            end           
        end
    end  
end

function [isok,idx] = inNodes(node,nodes)
    for i = 1:length(nodes)
        tnode = nodes(i);
        if node.xidx == tnode.xidx...
                && node.yidx == tnode.yidx...
                && node.yawidx == tnode.yawidx
            idx = i;
            isok = true;
            return
        end
    end
    idx = 1;
    isok = false;
end

function [isok,tnode] = CalcNextNode(wknode,D,delta,veh,cfg)
    px = wknode.x;
    py = wknode.y;
    pth = wknode.theta;
    gres = cfg.XY_GRID_RESOLUTION;
    obstline = cfg.ObstLine;
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1;
    x = zeros(1,nlist+1);
    y = x;
    th = x;
    x(1) = px;
    y(1) = py;
    th(1) = pth;
    for idx = 1:nlist
        [px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
        x(idx+1) = px;
        y(idx+1) = py;
        th(idx+1) = pth;
        if rem(idx,5) == 0
            tvec = [px,py,pth];
            isCollision = VehicleCollisionCheck(tvec,obstline,veh);
            if isCollision
                break
            end
        end
    end
    tnode = wknode;
    if isCollision
        isok = false;
        return
    else
%         plot(x,y);
        [isok,xidx,yidx,thidx] = CalcIdx(px,py,pth,cfg);
        if isok == false
            return
        else
            cost = wknode.cost;
            if D > 0
                cost = cost + gres*1.5;
            else
                cost = cost + cfg.BACK_COST*gres*1.5;
            end
            if D ~= wknode.D
                cost = cost + cfg.SB_COST;
            end
            cost = cost + cfg.STEER_COST*abs(delta);
            cost = cost + cfg.STEER_CHANGE_COST*abs(delta-wknode.delta);
            tnode = Node(xidx,yidx,thidx,D,delta,px,py,pth,...
                [wknode.xidx,wknode.yidx,wknode.yawidx],cost);
        end         
    end
end

function [wknode,nodes] = PopNode(nodes,cfg)
    mincost = inf;
    minidx = 1;
    gres = cfg.XY_GRID_RESOLUTION;
    for idx = 1:length(nodes)
        tnode = nodes(idx);
        % x in the col y in row
        tcost = TotalCost(tnode,cfg);
        if tcost < mincost
            mincost = tcost;
            minidx = idx;
        end
    end
    wknode = nodes(minidx);
    nodes(minidx) = [];
end

function cost = TotalCost(wknode,cfg)
    gres = cfg.XY_GRID_RESOLUTION;
    costmap = cfg.ObstMap;
    % from grid center to goal
    cost = cfg.H_COST*costmap(wknode.yidx,wknode.xidx);
    % from node position to grid center
    xshift = wknode.x - (gres*(wknode.xidx-0.5)+cfg.MINX);
    yshift = wknode.y - (gres*(wknode.yidx-0.5)+cfg.MINY);
    cost = cost+cfg.H_COST*norm([xshift,yshift]);
    % f = h + g
    cost = wknode.cost + cost;
end

function [isok,path] = AnalysticExpantion(Start,End,Vehicle,Configure)
    isok = true;
    isCollision = false;
    pvec = End-Start;
    x = pvec(1);
    y = pvec(2);
    phi = Start(3);
    phi = mod2pi(phi);
    dcm = angle2dcm(phi, 0, 0);
    % dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
    tvec = dcm*[x; y ; 0];
    x = tvec(1);
    y = tvec(2);
    veh = Vehicle;
    cfg = Configure;
    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    obstline = cfg.ObstLine;
    path = FindRSPath(x,y,pvec(3),veh);
    types = path.type;
    t = rmin*path.t;
    u = rmin*path.u;
    v = rmin*path.v;
    w = rmin*path.w;
    x = rmin*path.x;
    segs = [t,u,v,w,x];
    pvec = Start;
    for i = 1:5
        if segs(i) == 0
            continue
        end
        px =pvec(1);
        py = pvec(2);
        pth = pvec(3);
        s = sign(segs(i));
        D = s*mres;
        if types(i) == 'S'          
            delta = 0;
        elseif types(i) == 'L'
            delta = smax;
        elseif types(i) == 'R'
            delta = -smax;
        else
            % do nothing
        end
        for idx = 1:round(abs(segs(i))/mres)                
           	[px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
            if rem(idx,5) == 0
                tvec = [px,py,pth];
                isCollision = VehicleCollisionCheck(tvec,obstline,veh);
                if isCollision
                    break
                end
            end
        end
        if isCollision
            isok = false;
            break
        end
        pvec = [px,py,pth];
    end
    if (mod2pi(pth) - End(3)) > deg2rad(5)
        isok = false;
    end
end

function [x,y,theta] = VehicleDynamic(x,y,theta,D,delta,L)
    x = x+D*cos(theta);
    y = y+D*sin(theta);
    theta = theta+D/L*tan(delta);
    theta = mod2pi(theta);
end

function [isok,xidx,yidx,thidx] = CalcIdx(x,y,theta,cfg)
    gres = cfg.XY_GRID_RESOLUTION;
    yawres = cfg.YAW_GRID_RESOLUTION;
    xidx = ceil((x-cfg.MINX)/gres);
    yidx = ceil((y-cfg.MINY)/gres);
    theta = mod2pi(theta);
    thidx = ceil((theta-cfg.MINYAW)/yawres);
    isok = true;
    if xidx <=0 || xidx > ceil((cfg.MAXX-cfg.MINX)/gres)
        isok = false;
        return
    elseif yidx <=0 || yidx > ceil((cfg.MAXY-cfg.MINY)/gres)
        isok = false;
        return
    end
    costmap = cfg.ObstMap;
    if costmap(yidx,xidx) == inf
        isok = false;
    end
end

function v = mod2pi(x)
    v = rem(x,2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end