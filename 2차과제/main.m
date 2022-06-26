% 피타고라스의 정리를 이용해 드론이 제자리에 돌아오도록 설계했다.

% 드론 객체 선언
droneObj=ryze();

% 드론 띄우기
takeoff(droneObj);
pause(1.5);

% 왼쪽으로 이동(1.5m만큼 이동하도록 설계)
moveleft(droneObj,"Distance", 1.5,"Speed",0.5,"WaitUntilDone",true);
pause(1.5);

% 회전(시계방향으로 45도 회전 설계)
turn(droneObj,deg2rad(45));
pause(1.5);

% 앞으로 이동(1.5*sqrt(2)m만큼 이동하도록 설계)
moveforward(droneObj,"Distance", 1.5*sqrt(2),"Speed",0.5,"WaitUntilDone",true);
pause(1.5);

% 회전(시계방향으로 135도 회전 설계)
turn(droneObj,deg2rad(135));
pause(1.5);

% 앞으로 이동(1.5m만큼 이동하도록 설계)
moveforward(droneObj,"Distance", 1.5,"Speed",0.5,"WaitUntilDone",true);
pause(1.5);

% 드론 착륙
land(droneObj);
