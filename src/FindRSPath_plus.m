function path = FindRSPath_plus(x,y,phi,veh)
    rmin = veh.MIN_CIRCLE; %minimum turning radius
    x = x/rmin;
    y = y/rmin;
    reedsConnObj = reedsSheppConnection;
    startPose = [0 0 0];
    goalPose = [x y phi];
    [pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
    test = pathSegObj{1};
    type = [];
    value = zeros(5,1);
    for i = 1:5
        type = [type,test.MotionTypes{i}];
        value(i) = test.MotionDirections(i)*test.MotionLengths(i);
    end
    path = RSPath(type,value(1),value(2),value(3),value(4),value(5));
end