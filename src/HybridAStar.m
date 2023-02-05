function [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure)  
    veh = Vehicle;
    cfg = Configure;
    mres = cfg.MOTION_RESOLUTION; % motino resolution 
    
    % ����ʼ��λ��(x,y,theta)ת��Ϊgrid�ϵ�դ������
    [isok,xidx,yidx,thidx] = CalcIdx(Start(1),Start(2),Start(3),cfg);
    if isok % ��λ��դ����Ϊһ����㣬�γ�����ṹ
        tnode = Node(xidx,yidx,thidx,mres,0,Start(1),Start(2),Start(3),[xidx,yidx,thidx],0);
    end
    Open = [tnode]; % hybrid A*��Open����
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
        [isok1,idx] = inNodes(wknode,Close);
        
        % �ж��Ƿ���Close������
        if isok1
            Close(idx) = wknode;
        else
            Close = [Close, wknode];
        end

        % ��wknodeΪ���ڵ�������������ʹ��Reeds-Shepp�������ڳ�������ģ�ͽ����˶�ѧ������չ�ӽ��
        [isok2,path] = AnalysticExpantion([wknode.x,wknode.y,wknode.theta],End,veh,cfg);
        if  isok2
            %��wknode��idx�Ƶ�Close���������
            if isok1
                Close(end+1) = wknode;
                Close(idx) = [];
            else
            end
            [x,y,th,D,delta] = getFinalPath(path,Close,veh,cfg);
            break % �����ֱ�ӵõ�RS���ߣ�������whileѭ��
        end
        [Open,Close] = Update(wknode,Open,Close,veh,cfg); % ʹ��
    end
%     [isok,path] = AnalysticExpantion(Start,End,Vehicle,Configure);
end

function [x,y,th,D,delta] = getFinalPath(path,Close,veh,cfg)
    wknode = Close(end); % RS���������һ��Ԫ����Ŀ���
    Close(end) = [];
    nodes = [wknode];
    % ��Ŀ���wknode��parent,���ݣ�ֱ��Close����Ϊ��
    while ~isempty(Close)
        n = length(Close);
        parent = wknode.parent;
        % �����Ŀ�귵�ص���ʼ���·�������У�����nodes��
        for i = n:-1:1
            flag = 0; % ֻ�и�ֵ��û��ʹ��
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
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1; % �̶�ֵ31����դ���������ѡȡ�ߵĽ��㻹��ѡȡդ�������йأ�floor�ǳ����������������     % ÿ���켣�����2�׵ĳ��ȡ��������1.5��ȷ����һ��ĩ��״̬�϶�����һ��դ���У����ỹ��һ��դ���У��ڵ�ͼդ�����ӽ����չ������Խ��߳�����1.4����ʱ������ͬһ��դ����
    x = [];
    y = [];
    th = [];
    D = [];
    delta = [];
    flag = 0;
    % ·��Ҫô�Ǵ�RS·����Ҫô����RS·���ͻ��A*���һ������·�����ȴ�����A*�Ľ�㣬�����RS·�����϶���RS·��
    if length(nodes) >= 2
        % ���Ǵ���RS·����������RS·���ͻ��A*���һ������·����>=2��Щ�ڵ㶼�ǻ��A*�ѳ�����
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
            % ɾ���㣬ΪRS·����׼��
            if i ~= 2
                x(end) = [];
                y(end) = [];
                th(end) = [];
                D(end) = [];
                delta(end) = [];
            end
        end
    else
        % ���һ������Ǵ���RS·���յ�
        flag = 1;
        tnode = nodes(1);
        px = tnode.x;
        py = tnode.y;
        pth = tnode.theta;
        x = [x, px];
        y = [y, py];
        th = [th, pth];
    end
    % ��ʱ�Ѿ�������ȫ��·�������϶���һ����RS·��
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
        s = sign(segs(i)); % ǰ�������
        
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
        % ����RS����·�������룬�����˶�ѧ��ʽ����RS������ÿ��·�����״̬x,y,th
        for idx = 1:round(abs(segs(i))/mres) % ��������Ϊ�����С��������
           	[px,py,pth] = VehicleDynamic(px,py,pth,s*mres,tdelta,veh.WB); % s*mres��s����ǰ���ͺ���
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
    % all possible control input��
    for D = [-mres,mres] % D��0.1m,��������ǰ�������,������ǰλ�õĺ�����������һ��λ�õĺ�������֮���ֱ�߾��룬��2���ӽ��
        for delta = [-smax:sres:-sres,0,sres:sres:smax] % delta��ת��ǣ��ֱ�����0.03[rad]��[-0.6,0.6],��21���ӽ��(����0[rads])
            [isok,tnode] = CalcNextNode(wknode,D,delta,veh,cfg); % ����wknode�������ӽ�㣬һ��2*21=42�����˺����Ǹ��ݹ̶���D��delta����wknode����һ��·���������ӽ�㣬tnode�Ǵ���·����ĩ�˵�
            if isok == false % �ӽ�㲻����
                continue
            end
            [isok,~] = inNodes(tnode,Close);% ��Close������
            if isok
                continue
            end 
            % ��չ�Ľڵ������Open�бȽ�fֵ;����������ӵ�Open��
            [isok,idx] = inNodes(tnode,Open);
            if isok
                % ��֮ǰ��cost�Ƚϣ����и���
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

% ����D��delta����wknode����һ��·���������ӽ��
function [isok,tnode] = CalcNextNode(wknode,D,delta,veh,cfg)
    px = wknode.x;
    py = wknode.y;
    pth = wknode.theta;
    gres = cfg.XY_GRID_RESOLUTION;
    obstline = cfg.ObstLine;
    % ÿ���켣�����2�׵ĳ��ȡ��������1.5��ȷ����һ��ĩ��״̬�϶�����һ��դ���У����ỹ��һ��դ���У��ڵ�ͼդ�����ӽ����չ������Խ��߳�����1.4����ʱ������ͬһ��դ����
    nlist = floor(gres*1.5/cfg.MOTION_RESOLUTION)+1; %�˴��Ƕ�ֵ31�� �������D��delta�£�����һ��·����wknode���ӽ�����Ŀ���Ա��������     % ÿ���켣�����2�׵ĳ��ȡ��������1.5��ȷ����һ��ĩ��״̬�϶�����һ��դ���У����ỹ��һ��դ���У��ڵ�ͼդ�����ӽ����չ������Խ��߳�����1.4����ʱ������ͬһ��դ����
    x = zeros(1,nlist+1);
    y = x;
    th = x;
    x(1) = px;
    y(1) = py;
    th(1) = pth;
    for idx = 1:nlist % ���ݵ�ǰ��״̬�͸����Ŀ��ƣ��������·���ϵ����ŵĳ���״̬��������һʱ�̼�����һʱ��
        [px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
        x(idx+1) = px; % x,y,th���������ݣ�����û�õ�����
        y(idx+1) = py;
        th(idx+1) = pth;
        if rem(idx,5) == 0 % ÿ��5�������һ����ײ���
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
        [isok,xidx,yidx,thidx] = CalcIdx(px,py,pth,cfg); % ��·��ĩ�˵��ʵ������ת��Ϊդ������
        if isok == false
            return
        else
            cost = wknode.cost;
            if D > 0 % ǰ��
                cost = cost + gres*1.5; % ÿ���켣�����2�׵ĳ��ȡ��������1.5��ȷ����һ��ĩ��״̬�϶�����һ��դ���У����ỹ��һ��դ���У��ڵ�ͼդ�����ӽ����չ������Խ��߳�����1.4����ʱ������ͬһ��դ����
            else % ����
                cost = cost + cfg.BACK_COST*gres*1.5;
            end
            if D ~= wknode.D
                cost = cost + cfg.SB_COST;
            end
            cost = cost + cfg.STEER_COST*abs(delta);
            cost = cost + cfg.STEER_CHANGE_COST*abs(delta-wknode.delta);
            tnode = Node(xidx,yidx,thidx,D,delta,px,py,pth,...
                [wknode.xidx,wknode.yidx,wknode.yawidx],cost); % tnode��·����ĩ�˵㣬costΪ����ǰ״̬����·��ĩ��״̬�ĳɱ�
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
    % ��դ�����ĵ�Ŀ��
    cost = cfg.H_COST*costmap(wknode.yidx,wknode.xidx); % ���ϰ���ײ��ͼ�ϵĳɱ�����A*�ѳ����ģ���ά��ͼ��û�ú���
    % �ӵ�ǰ��㵽դ������
    xshift = wknode.x - (gres*(wknode.xidx-0.5)+cfg.MINX); % դ���index���ߵĽ��㣬������դ�������,��������ʱ���Ի��м�0.5
    yshift = wknode.y - (gres*(wknode.yidx-0.5)+cfg.MINY);
    cost = cost+cfg.H_COST*norm([xshift,yshift]);
    % f = g + h
    cost = wknode.cost + cost;
end

function [isok,path] = AnalysticExpantion(Start,End,Vehicle,Configure)
    isok = true;
    isCollision = false;
    
    % �����ת����ԭ�����켣���任����ϵ��
    pvec = End-Start;
    x = pvec(1);
    y = pvec(2);
    phi = Start(3);
    phi = mod2pi(phi);
    dcm = angle2dcm(phi, 0, 0); % ���start����ϵ�ڻ�����ϵ�µķ������Ҿ���
    % dcm*x ��ʾ���������е�x��ʾ����ת�������ϵ�У�������������ת����������������еı�ʾ
    tvec = dcm * [x; y ; 0]; % ����������ת������������start�����еı�ʾ
    x = tvec(1);
    y = tvec(2);
    veh = Vehicle;
    cfg = Configure;
    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    obstline = cfg.ObstLine;
    
    % ���Ƿ�ӵ�ǰ�㵽Ŀ����������ײ��Reeds-Shepp�켣��ǰ��pvec=End-Start;������������ע�⣡�����x,y,prev(3)�ǰ����ת������startΪԭ������ϵ�µ�����
    path = FindRSPath_plus(x,y,pvec(3),veh);

    % �����Ǹ���·����ͳ����˶�ѧģ�ͼ���λ�ã�����Ƿ�������ײ������isok��ֵ����ÿ��·������㵽�յ㰴˳����д�����һ���߶ε��յ�pvec����һ���߶ε����px,py,pth��  
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
        s = sign(segs(i)); % ���ź���,�жϴ˶��˶�������ǰ�����Ǻ���
        
        % ���ݳ�����2*3���˶�����(ǰ��2�֣�ת��3��)������D��delta
        D = s*mres; % �ֱ��ʵ�����
        if types(i) == 'S'
            delta = 0;
        elseif types(i) == 'L'
            delta = smax;
        elseif types(i) == 'R'
            delta = -smax;
        else
            % do nothing
        end
        
        % �Ѵ˶ε�·����ɢ��Ϊ·�㣬��դ������,Ȼ��Ϊ·�㣬Ȼ�����Ƿ�����ϰ�����ײ����
        for idx = 1:round(abs(segs(i))/mres) % round()��������
            % D��delta�ǹ̶���˵��ת���ʱ���ǰ��̶��뾶��Բת��
           	[px,py,pth] = VehicleDynamic(px,py,pth,D,delta,veh.WB);
            if rem(idx,5) == 0 % rem(a,b)�������� a/b���������ÿ5���㣬��0.5m������Ƿ���ײ
                tvec = [px,py,pth];
                isCollision = VehicleCollisionCheck(tvec,obstline,veh);
                if isCollision
                    break
                end
            end
         end
        if isCollision
            isok = false;
            break % ���·��������ײ����������Reeds-Shepp·��
        end
        pvec = [px,py,pth];
    end
    % �յ�λ��С��������ֵҲ����
    if (mod2pi(pth) - End(3)) > deg2rad(5)
        isok = false;
    end
end

% ���ݵ�ǰλ�˺�����,������һλ�õ�λ��
function [x,y,theta] = VehicleDynamic(x,y,theta,D,delta,L)
    x = x+D*cos(theta); % �˶�ѧ��ʽ�� x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta),�ڲ���ʱ��t��,����x = x + v_x * t * cos(theta)������v_x * t=D
    y = y+D*sin(theta); % �˶�ѧ��ʽ
    theta = theta+D/L*tan(delta); % L�����,����仯,theta_dot=v/R,R=L/tan(delta)
    theta = mod2pi(theta);
end

% ��λ��(x,y,theta)ת��Ϊgrid�ϵ�դ������,���������ʵ����isok=false
function [isok,xidx,yidx,thidx] = CalcIdx(x,y,theta,cfg)
    gres = cfg.XY_GRID_RESOLUTION;
    yawres = cfg.YAW_GRID_RESOLUTION;
    xidx = ceil((x-cfg.MINX)/gres);
    yidx = ceil((y-cfg.MINY)/gres);
    theta = mod2pi(theta); % ����theta��Χ��[-pi,pi]����
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

% �ѻ���x������[-pi,pi]
function v = mod2pi(x)
    v = rem(x,2*pi); % ������x/2pi������
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end