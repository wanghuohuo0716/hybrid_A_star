function VehicleAnimation(x,y,theta,cfg,veh)
    sz=get(0,'screensize');
    figure('outerposition',sz);
    videoFWriter = VideoWriter('Parking.mp4','MPEG-4');
    open(videoFWriter);
    ObstList = cfg.ObstList;
    scatter(ObstList(:,1),ObstList(:,2),10,'r')
    hold on
    axis equal
    xlim([cfg.MINX,cfg.MAXX]);
    ylim([cfg.MINY,cfg.MAXY]);
    plot(x,y,'b')    
    px = x(1);
    py = y(1);
    pth = theta(1);
    [vehx,vehy] = getVehTran(px,py,pth,veh);
    h1 = plot(vehx,vehy,'k');
    h2 = plot(px,px,'rx','MarkerSize',10);
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2:length(theta)
        px = x(i);
        py = y(i);
        pth = theta(i);
        [vehx,vehy] = getVehTran(px,py,pth,veh);
        h1.XData = vehx;
        h1.YData = vehy;
        h2.XData = px;
        h2.YData = py;
        img = getframe(gcf);
        writeVideo(videoFWriter,img);
%         pause(0.005)
    end
    close(videoFWriter);
end

function [x,y] = getVehTran(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    Cornerfl = [LF, W/2];
    Cornerfr = [LF, -W/2];
    Cornerrl = [-LB, W/2];
    Cornerrr = [-LB, -W/2];
    Pos = [x,y];
    dcm = angle2dcm(-theta, 0, 0);
    tvec = dcm*[Cornerfl';0];
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos;
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end