% [x1 x2 
% [y1 y2
% [th1 th2

DEG2RAD = 3.14/180;
poseSize = 4;

goalPoseArray = [0 1 2 3; 
 0 0 1 2;
 0 0 -45 -45];

%find dXw,dYw,dthetaw
deltaXw = goalPoseArray(1,2:poseSize) - goalPoseArray(1,1:poseSize-1);
deltaYw = goalPoseArray(2,2:poseSize) - goalPoseArray(2,1:poseSize-1);
deltaTw = goalPoseArray(3,2:poseSize) - goalPoseArray(3,1:poseSize-1);

%find dXr, dYr, dthetar
deltaXr = ( cos(goalPoseArray(3,1:poseSize-1)*DEG2RAD) .* (deltaXw) + sin(goalPoseArray(3,1:poseSize-1)*DEG2RAD) .* (deltaYw) ) ;

%move robot

%print world position
