function [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure)  
    veh = Vehicle;
    cfg = Configure;
    mres = cfg.MOTION_RESOLUTION; % motino resolution 
    
    % 把起始的位姿(x,y,theta)转换为grid上的栅格索引
    [isok,xidx,yidx,thidx] = CalcIdx(Start(1),Start(2),Start(3),cfg);
    if isok % 把位姿栅格定义为一个结点，形成链表结构
        tnode = Node(xidx,yidx,thidx,mres,0,Start(1),Start(2),Start(3),[xidx,yidx,thidx],0);
    end
    Open = [tnode]; % hybrid A*的Open集合
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
        
        % 判断是否在Close集合内
        if isok
            Close(idx) = wknode;
        else
            Close = [Close, wknode];
        end

        % 以wknode为根节点生成搜索树，使用Reeds-Shepp方法基于车辆单轨模型进行运动学解析拓展子结点
        [isok,path] = AnalysticExpantion([wknode.x,wknode.y,wknode.theta],End,veh,cfg);
        if  isok
            %把wknode从idx移到Close集合最后面
            Close(end+1) = wknode;
            Close(idx) = [];
            [x,y,th,D,delta] = getFinalPath(path,Close,veh,cfg);
            break % 如果能直接得到RS曲线，则跳出while循环
        end
        [Open,Close] = Update(wknode,Open,Close,veh,cfg); % 使用
    end
%     [isok,path] = AnalysticExpantion(Start,End,Vehicle,Configure);
end

function [x,y,th,D,delta] = getFinalPath(path,Close,veh,cfg)
    wknode = Close(end); % RS曲线中最后一个元素是目标点
    Close(end) = [];
    nodes = [wknode];
    % 找目标点wknode的parent,回溯，直到Close集合为空
    while ~isempty(Close)
        n = length(Close);
        parent = wknode.parent;
        % 计算从目标返回到起始点的路径点序列，放入nodes中
        for i = n:-1:1
            flag = 0; % 只有赋值，没有使用
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
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1; % 固定值31，和栅格的索引是选取线的交点还是选取栅格中心有关，floor是朝负无穷大四舍五入     % 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
    x = [];
    y = [];
    th = [];
    D = [];
    delta = [];
    flag = 0;
    % 路径要么是纯RS路径，要么是由RS路径和混合A*组合一起来的路径，先处理混合A*的结点，最后处理RS路径，肯定有RS路径
    if length(nodes) >= 2
        % 不是纯的RS路径，而是由RS路径和混合A*组合一起来的路径，>=2这些节点都是混合A*搜出来的
        for i = length(nodes):-1:2
            tnode = nodes(i);
            ttnode = nodes(i-1); % parent of i
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
            % 删除点，为RS路径做准备
            if i ~= 2
                x(end) = [];
                y(end) = [];
                th(end) = [];
                D(end) = [];
                delta(end) = [];
            end
        end
    else
        % 最后一个结点是纯的RS路径终点
        flag = 1;
        tnode = nodes(1);
        px = tnode.x;
        py = tnode.y;
        pth = tnode.theta;
        x = [x, px];
        y = [y, py];
        th = [th, pth];
    end
    % 此时已经搜完了全部路径，最后肯定有一段是RS路径
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
        s = sign(segs(i)); % 前进或后退
        
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
        % 根据RS曲线路径的输入，基于运动学公式计算RS曲线上每个路径点的状态x,y,th
        for idx = 1:round(abs(segs(i))/mres) % 四舍五入为最近的小数或整数
           	[px,py,pth] = VehicleDynamic(px,py,pth,s*mres,tdelta,veh.WB); % s*mres中s代表前进和后退
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
    smax = veh.MAX_STEER; % 0.6[rad],maximum steering angle
    sres = smax/cfg.N_STEER; % 20,steering resolution  
    % all possible control input，
    for D = [-mres,mres] % D是0.1m,正负代表前进或后退,车辆当前位置的后轴中心与下一个位置的后轴中心之间的直线距离，有2个子结点
        for delta = [-smax:sres:-sres,0,sres:sres:smax] % delta是转向角，分辨率是0.03[rad]，[-0.6,0.6],有21个子结点(包含0[rads])
            [isok,tnode] = CalcNextNode(wknode,D,delta,veh,cfg); % 计算wknode的所有子结点，一共2*21=42个，此函数是根据固定的D和delta计算wknode沿着一条路径的所有子结点，tnode是此条路径的末端点
            if isok == false % 子结点不可行
                continue
            end
            [isok,~] = inNodes(tnode,Close);% 在Close集合中
            if isok
                continue
            end 
            % 拓展的节点如果在Open中比较f值;若不在则添加到Open中
            [isok,idx] = inNodes(tnode,Open);
            if isok
                % 与之前的cost比较，进行更新
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

% 根据D和delta计算wknode沿着一条路径的所有子结点
function [isok,tnode] = CalcNextNode(wknode,D,delta,veh,cfg)
    px = wknode.x;
    py = wknode.y;
    pth = wknode.theta;
    gres = cfg.XY_GRID_RESOLUTION;
    obstline = cfg.ObstLine;
    % 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1; %此处是定值31， 计算给定D和delta下，沿着一条路径上wknode的子结点的数目，以便填充数据     % 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
    x = zeros(1,nlist+1);
    y = x;
    th = x;
    x(1) = px;
    y(1) = py;
    th(1) = pth;
    for idx = 1:nlist % 根据当前的状态和给定的控制，计算此条路径上的连着的车辆状态，根据上一时刻计算下一时刻
        [px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
        x(idx+1) = px; % x,y,th储存了数据，但是没用到变量
        y(idx+1) = py;
        th(idx+1) = pth;
        if rem(idx,5) == 0 % 每隔5个点进行一次碰撞检测
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
        [isok,xidx,yidx,thidx] = CalcIdx(px,py,pth,cfg); % 把路径末端点的实际坐标转换为栅格坐标
        if isok == false
            return
        else
            cost = wknode.cost;
            if D > 0 % 前进
                cost = cost + gres*1.5; % 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
            else % 后退
                cost = cost + cfg.BACK_COST*gres*1.5;
            end
            if D ~= wknode.D
                cost = cost + cfg.SB_COST;
            end
            cost = cost + cfg.STEER_COST*abs(delta);
            cost = cost + cfg.STEER_CHANGE_COST*abs(delta-wknode.delta);
            tnode = Node(xidx,yidx,thidx,D,delta,px,py,pth,...
                [wknode.xidx,wknode.yidx,wknode.yawidx],cost); % tnode是路径的末端点，cost为到当前状态到此路径末端状态的成本
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
    % 从栅格中心到目标
    cost = cfg.H_COST*costmap(wknode.yidx,wknode.xidx); % 无障碍碰撞地图上的成本，用A*搜出来的，二维地图，没用航向
    % 从当前结点到栅格中心
    xshift = wknode.x - (gres*(wknode.xidx-0.5)+cfg.MINX); % 栅格的index是线的交点，而不是栅格的中心,在求坐标时所以会有减0.5
    yshift = wknode.y - (gres*(wknode.yidx-0.5)+cfg.MINY);
    cost = cost+cfg.H_COST*norm([xshift,yshift]);
    % f = g + h
    cost = wknode.cost + cost;
end

function [isok,path] = AnalysticExpantion(Start,End,Vehicle,Configure)
    isok = true;
    isCollision = false;
    
    % 将起点转换到原点计算轨迹，变换坐标系了
    pvec = End-Start;
    x = pvec(1);
    y = pvec(2);
    phi = Start(3);
    phi = mod2pi(phi);
    dcm = angle2dcm(phi, 0, 0); % 起点start坐标系在基坐标系下的方向余弦矩阵
    % dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
    tvec = dcm * [x; y ; 0]; % 计算坐标旋转后各向量在起点start坐标中的表示
    x = tvec(1);
    y = tvec(2);
    veh = Vehicle;
    cfg = Configure;
    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    obstline = cfg.ObstLine;
    
    % 看是否从当前点到目标点存在无碰撞的Reeds-Shepp轨迹，前面pvec=End-Start;的意义就在这里，注意！这里的x,y,prev(3)是把起点转换成以start为原点坐标系下的坐标
    path = FindRSPath(x,y,pvec(3),veh);

    % 以下是根据路径点和车辆运动学模型计算位置，检测是否会产生碰撞，返回isok的值。对每段路径从起点到终点按顺序进行处理，这一个线段的终点pvec是下一个线段的起点px,py,pth，  
    types = path.type;
    t = rmin*path.t;
    u = rmin*path.u;
    v = rmin*path.v;
    w = rmin*path.w;
    x = rmin*path.x;
    segs = [t,u,v,w,x];
    pvec = Start;
    for i = 1:5
        if segs(i) ==0
            continue
        end
        px =pvec(1);
        py = pvec(2);
        pth = pvec(3);
        s = sign(segs(i)); % 符号函数,判断此段运动方向是前进还是后退
        
        % 根据车辆的2*3种运动类型(前后2种，转向3种)，设置D和delta
        D = s*mres; % 分辨率的正负
        if types(i) == 'S'
            delta = 0;
        elseif types(i) == 'L'
            delta = smax;
        elseif types(i) == 'R'
            delta = -smax;
        else
            % do nothing
        end
        
        % 把此段的路径离散成为路点，即栅格索引,然后为路点，然后检测是否存在障碍物碰撞问题
        for idx = 1:round(abs(segs(i))/mres) % round()四舍五入
            % D和delta是固定，说明转弯的时候是按固定半径的圆转弯
           	[px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
            if rem(idx,5) == 0 % rem(a,b)，返回用 a/b后的余数，每5个点，即0.5m检查下是否碰撞
                tvec = [px,py,pth];
                isCollision = VehicleCollisionCheck(tvec,obstline,veh);
                if isCollision
                    break
                end
            end
         end
        if isCollision
            isok = false;
            break % 如果路径存在碰撞则舍弃此条Reeds-Shepp路径
        end
        pvec = [px,py,pth];
    end
    % 终点位姿小于期望阈值也舍弃
    if (mod2pi(pth) - End(3)) > deg2rad(5)
        isok = false;
    end
end

% 根据当前位姿和输入,计算下一位置的位姿
function [x,y,theta] = VehicleDynamic(x,y,theta,D,delta,L)
    x = x+D*cos(theta); % 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta),在采样时间t内,则有x = x + v_x * t * cos(theta)，其中v_x * t=D
    y = y+D*sin(theta); % 运动学公式
    theta = theta+D/L*tan(delta); % L是轴距,航向变化,theta_dot=v/R,R=L/tan(delta)
    theta = mod2pi(theta);
end

% 把位姿(x,y,theta)转换为grid上的栅格索引,如果不符合实际则isok=false
function [isok,xidx,yidx,thidx] = CalcIdx(x,y,theta,cfg)
    gres = cfg.XY_GRID_RESOLUTION;
    yawres = cfg.YAW_GRID_RESOLUTION;
    xidx = ceil((x-cfg.MINX)/gres);
    yidx = ceil((y-cfg.MINY)/gres);
    theta = mod2pi(theta); % 控制theta范围在[-pi,pi]区间
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

% 把弧度x控制在[-pi,pi]
function v = mod2pi(x)
    v = rem(x,2*pi); % 求整除x/2pi的余数
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end