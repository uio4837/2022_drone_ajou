stage_pixel=0;      % 각 색의 픽셀값을 읽어오는 변수
count_forward =0;   % 전진한 횟수를 세주는 변수

area_meas=0;        % 각 스테이지 별 원의 면적

center_place=[480,220]; % centroid의 값이 다음 값과 인접할 때 전진시키기 위해 사용 기존 210
centroid=zeros(size(center_place)); % 원의 중심 좌표를 읽어오는 변수

drone=ryze();       % 드론 객체 drone 선언
cam=camera(drone);  % 드론 객체 drone의 카메라 객체 cam 선언
takeoff(drone);

moveup(drone,'Distance',0.5);   % 1st stage의 시간 단축을 위해 드론 객체를 위로 0.5m 이동

% 1st stage
while stage_pixel<200   % 해당 색의 픽셀값이 200보다 작을때 while문 내의 명령들을 수행
    frame=snapshot(cam);    % snapshot 함수를 통해 카메라 객체 cam의 사진을 frame에 저장
    img = double(frame);    % frame의 값들을 double형으로 바꿔준 뒤, img 변수에 저장
    [R, C, X]=size(img);    % img 변수의 크기(행, 열, 색의 3차원 배열)을 각각 R, C, X에 저장
    img2=zeros(size(img));         % img2 변수를 사용하기 위해 사전할당을 통해 처리
    img3=zeros(size(img));         % img3 변수를 사용하기 위해 사전할당을 통해 처리

    % 행, 열에 값을 대입해주는 것이기에 이중 for문을 통해 조건에 따라 다른 값들을 img2에 대입
    for i =1:R
        for j=1:C
            if img(i,j,1) - img(i,j,2) > -5 || img(i,j,1) - img(i,j,3) > -5|| img(i,j,2) - img(i,j,3) > -40 % 파란색 링 색깔을 인식하기 위한 조건
                % 해당 조건에 해당하는 경우, img2의 R, G, B 모든 요소의 값에 최대치인 255를 대입
                img2(i,j,1) = 255;  
                img2(i,j,2) = 255;
                img2(i,j,3) = 255;

            else
                % 그 외의 경우, img2의 R, G, B 모든 요소의 값에 0을 대입
                img2(i,j,:) = 0;
                img2(i,j,2) = 0;
                img2(i,j,3) = 0;
            end

        end
    end
    
    % 파란색 크로마키 내의 구멍의 원을 그리는 작업
    circle_ring = img2/255; 
    circle_ring_Gray = rgb2gray(circle_ring);
    circle_ring_bi=imbinarize(circle_ring_Gray);
    bi2=imcomplement(circle_ring_bi);
    bw = bwareaopen(bi2,8000);
    bw = imcomplement(bw);
    se = strel('disk',10);
    bw2 = imclose(bw,se);
    bw3 = bwareaopen(bw2,8000);
    [B,L] = bwboundaries(bw3,'noholes');

    % figure 1에 작업을 완료한 bw3를 그리고, 축을 표시한 다음 덧붙일 수 있도록 설계
    figure(1),imshow(bw3);
    axis on
    hold on

    % 원의 경계를 하얀색 선으로 plot
    for k = 1:length(B)
        boundary = B{k};
        plot(boundary(:,2),boundary(:,1),'w','LineWidth',2);
    end
    
    % 그려진 각 영역의 Area(면적)과 Centroid(중점) 정보를 저장하는 stats 선언
    stats = regionprops(L,'Area','Centroid');

    % 원이라고 판단하는 기준 수치 threshold 값을 0.7로 설정
    threshold = 0.7;

    % loop over the boundaries
    for k = 1:length(B)

        % obtain (X,Y) boundary coordinates corresponding to label 'k'
        boundary = B{k};

        % compute a simple estimate of the object's perimeter
        delta_sq = diff(boundary).^2;
        perimeter = sum(sqrt(sum(delta_sq,2)));

        % obtain the area calculation corresponding to label 'k'
        area = stats(k).Area;

        % compute the roundness metric
        metric = 4*pi*area/perimeter^2;

        % display the results
        metric_string = sprintf('%2.2f',metric);

        % 기준 수치인 threshold보다 클 경우 아래의 명령을 수행
        if metric > threshold
            area_meas=stats(k).Area;        % 해당 영역의 면적을 area_meas에 저장
            centroid = stats(k).Centroid;   % 해당 영역의 중점을 centroid에 저장
            plot(centroid(1),centroid(2),'r');  % centroid(중점)을 figure 1에 plot 
        end

        text(boundary(1,2)-35,boundary(1,1)+13,metric_string,'Color','y',...
            'FontSize',14,'FontWeight','bold')

    end

    % 3-1) 원의 중심 찾기 알고리즘
    Dis=centroid-center_place;  % 현재 원의 위치에서 기준이 되는 중점의 위치의 차이를 Dis로 저장

    % case 1
    if(abs(Dis(1))<28 && abs(Dis(2))<28)    % x 좌표 차이, y 좌표 차이가 27보다 작을 경우 앞으로 전진
        disp("Moving the drone forward"); 

        % 거리에 따른 원의 넓이를 실험적으로 측정했고, 그에 따른 전진 거리를 설정
        % 그 거리를 이동하고, 이동한 횟수를 1회로 설정
        if 50000<=area_meas && area_meas<60000
            disp("3.2m moveforward");
            moveforward(drone,'Distance',3.2,'Speed',0.8);
            count_forward = 1;          

        elseif 60000<=area_meas && area_meas<74000
            disp("3.0m moveforward");
            moveforward(drone,'Distance',3.0,'Speed',0.8);
            count_forward = 1;


        elseif 74000<=area_meas && area_meas<85000
            disp("2.8m moveforward");
            moveforward(drone,'Distance',2.8,'Speed',0.8);
            count_forward = 1;


        elseif 85000<=area_meas && area_meas<105000
            disp("2.6m moveforward");
            moveforward(drone,'Distance',2.6,'Speed',0.8);
            count_forward = 1;


        elseif 105000<=area_meas && area_meas<130000
            disp("2.4m moveforward");
            moveforward(drone,'Distance',2.4,'Speed',0.8);
            count_forward = 1;


        elseif 130000<=area_meas && area_meas<165000
            disp("2.2m moveforward");
            moveforward(drone,'Distance',2.2,'Speed',0.8);
            count_forward = 1;


        elseif 160000<=area_meas && area_meas<220000
            disp("2.0m moveforward");
            moveforward(drone,'Distance',2,'Speed',0.8);
            count_forward = 1;


        elseif 220000<=area_meas && area_meas<360000
            disp("1.8m moveforward");
            moveforward(drone,'Distance',1.8,'Speed',0.8);
            count_forward = 1;


        elseif 360000<=area_meas && area_meas<460000
            disp("1.6m moveforward");
            moveforward(drone,'Distance',1.6,'Speed',0.8);
            count_forward = 1;


        elseif 460000<=area_meas && area_meas<600000
            disp("1.4m moveforward");
            moveforward(drone,'Distance',1.4,'Speed',0.8);
            count_forward = 1;


        elseif 600000<=area_meas
            disp("1.2m moveforward");
            moveforward(drone,'Distance',1.2,'Speed',0.8);
            count_forward = 1;

        else
            disp("3.4m moveforward");
            moveforward(drone,'Distance',3.4,'Speed',0.8);
            count_forward = 1;
        end
        
        % 전진 후, 이미지를 다시 읽어오는 과정을 수행
        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) > 20 || img(i,j,1) - img(i,j,3) > 10|| img(i,j,2) - img(i,j,3) < 15  % 초록색이 아닌 색들을 없애기 위한 조건
                    % 초록색이 아닌 색들은 모두 제거해야 하므로 이 경우 img3의 R, G, B 모든 요소의 값을 0을 대입
                    img3(i,j,1) = 0;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;

                else    % 위의 경우가 아닌 경우, 모두 초록색이라 판단하고
                    % 그 외의 경우, img3의 R, G, B 요소들 중 G 요소에 최대 수치인 255를 대입
                    img3(i,j,1) = 0;
                    img3(i,j,2) = 255;
                    img3(i,j,3) = 0;
                    stage_pixel=stage_pixel+1;  % 각 색의 픽셀값을 읽어오는 변수인 stage_pixel의 값을 1 증가시킴

                end
            end
        end

        % case 2
    elseif(Dis(1)>0 && abs(Dis(1))>28 && Dis(2)<28)
        disp("Moving the drone right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 3
    elseif(Dis(1)<0 && abs(Dis(1))>28 && Dis(2)<28)
        disp("Moving the drone left");
        moveleft(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 4
    elseif(abs(Dis(1))<28 && Dis(2)>0 && abs(Dis(2))>28)
        disp("Moving the drone down");
        movedown(drone,'Distance',0.2,'Speed',1);
        pause(1.5);
    
        % case 5
    elseif(abs(Dis(1))<28 && Dis(2)<0 && abs(Dis(2))>28)
        disp("Moving the drone up");
        moveup(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 6
    elseif(Dis(1)>0 && abs(Dis(1))>28)
        disp("Moving right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 7
    elseif(Dis(1)<0 && abs(Dis(1))>28)
        disp("Moving left");
        moveleft(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % 나머지 경우 처리
    else
        disp("Hovering");

        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) > 20 || img(i,j,1) - img(i,j,3) > 10|| img(i,j,2) - img(i,j,3) < 15

                    img3(i,j,1) = 0;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;

                else

                    img3(i,j,1) = 0;
                    img3(i,j,2) = 255;
                    img3(i,j,3) = 0;
                    stage_pixel=stage_pixel+1;

                end
            end
        end
    end
    % 전진한 횟수가 1회인 경우 해당 while loop를 빠져나오도록 설정
    if (stage_pixel<200 && count_forward==1)
        break;
    end
end

disp("find_green");
turn(drone, deg2rad(90));
moveup(drone,'Distance',0.6);
moveforward(drone,'Distance',1.1,'Speed',0.8);
stage_pixel=0;
count_forward=0;

% 2nd stage
while stage_pixel<200
    frame=snapshot(cam);
    img = double(frame);
    [R, C, X]=size(img);    
    img2=zeros(size(img));         % img2 변수를 사용하기 위해 사전할당을 통해 처리
    img3=zeros(size(img));         % img3 변수를 사용하기 위해 사전할당을 통해 처리
    for i =1:R
        for j=1:C
            if img(i,j,1) - img(i,j,2) > -5 || img(i,j,1) - img(i,j,3) > -5|| img(i,j,2) - img(i,j,3) > -40
                img2(i,j,1) = 255;
                img2(i,j,2) = 255;
                img2(i,j,3) = 255;

            else
                img2(i,j,:) = 0;
                img2(i,j,2) = 0;
                img2(i,j,3) = 0;
            end

        end
    end

    circle_ring = img2/255;
    circle_ring_Gray = rgb2gray(circle_ring);
    circle_ring_bi=imbinarize(circle_ring_Gray);
    bi2=imcomplement(circle_ring_bi);
    bw = bwareaopen(bi2,8000);
    bw = imcomplement(bw);
    se = strel('disk',10);
    bw2 = imclose(bw,se);
    bw3 = bwareaopen(bw2,8000);
    [B,L] = bwboundaries(bw3,'noholes');

    figure(1),imshow(bw3);
    axis on
    hold on

    for k = 1:length(B)
        boundary = B{k};
        plot(boundary(:,2),boundary(:,1),'w','LineWidth',2);
    end

    stats = regionprops(L,'Area','Centroid');

    threshold = 0.7;

    % loop over the boundaries
    for k = 1:length(B)

        % obtain (X,Y) boundary coordinates corresponding to label 'k'
        boundary = B{k};

        % compute a simple estimate of the object's perimeter
        delta_sq = diff(boundary).^2;
        perimeter = sum(sqrt(sum(delta_sq,2)));

        % obtain the area calculation corresponding to label 'k'
        area = stats(k).Area;

        % compute the roundness metric
        metric = 4*pi*area/perimeter^2;

        % display the results
        metric_string = sprintf('%2.2f',metric);

        % mark objects above the threshold with a black circle
        if metric > threshold
            area_meas=stats(k).Area;
            centroid = stats(k).Centroid;
            plot(centroid(1),centroid(2),'b');
        end

        text(boundary(1,2)-35,boundary(1,1)+13,metric_string,'Color','y',...
            'FontSize',14,'FontWeight','bold')

    end

    Dis=centroid-center_place;

    % case 1
    if(abs(Dis(1))<27 && abs(Dis(2))<27)
        disp("Moving the drone forward");
        if 24500<=area_meas && area_meas<27000
            disp("3.2m moveforward");
            moveforward(drone,'Distance',3.2,'Speed',0.8);
            count_forward = 1;
           
        elseif 27000<=area_meas && area_meas<32000
            disp("3.0m moveforward");
            moveforward(drone,'Distance',3.0,'Speed',0.8);
            count_forward = 1;        

        elseif 32000<=area_meas && area_meas<39000
            disp("2.8m moveforward");
            moveforward(drone,'Distance',2.8,'Speed',0.8);
            count_forward = 1;
        
        elseif 39000<=area_meas && area_meas<47500
            disp("2.6m moveforward");
            moveforward(drone,'Distance',2.6,'Speed',0.8);
            count_forward = 1;
        
        elseif 47500<=area_meas && area_meas<62000
            disp("2.4m moveforward");
            moveforward(drone,'Distance',2.4,'Speed',0.8);
            count_forward = 1;
        
        elseif 62000<=area_meas && area_meas<84000
            disp("2.2m moveforward");
            moveforward(drone,'Distance',2.2,'Speed',0.8);
            count_forward = 1;
        
        elseif 84000<=area_meas && area_meas<92000
            disp("2.0m moveforward");
            moveforward(drone,'Distance',2,'Speed',0.8);
            count_forward = 1;
        
        elseif 92000<=area_meas && area_meas<140000
            disp("1.8m moveforward");
            moveforward(drone,'Distance',1.8,'Speed',0.8);
            count_forward = 1;
        
        elseif 140000<=area_meas && area_meas<230000
            disp("1.6m moveforward");
            moveforward(drone,'Distance',1.6,'Speed',0.8);
            count_forward = 1;
        
        elseif 230000<=area_meas && area_meas<380000
            disp("1.4m moveforward");
            moveforward(drone,'Distance',1.4,'Speed',0.8);
            count_forward = 1;
       
        elseif 380000<=area_meas
            disp("1.2m moveforward");
            moveforward(drone,'Distance',1.2,'Speed',0.8);
            count_forward = 1;
        
        else
            disp("3.4m moveforward");
            moveforward(drone,'Distance',3.4,'Speed',0.8);
            count_forward = 1;
        end

        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) < 11 || img(i,j,1) - img(i,j,3) > 0|| img(i,j,2) - img(i,j,3) > 20

                    img3(i,j,1) = 0;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;

                else

                    img3(i,j,1) = 122;
                    img3(i,j,2) = 48;
                    img3(i,j,3) = 160;
                    stage_pixel=stage_pixel+1;

                end
            end
        end

        % case 2
    elseif(Dis(1)>0 && abs(Dis(1))>27 && Dis(2)<27)
        disp("Moving the drone right");
        moveright(drone,'Distance',0.2,'Speed',1);        
        pause(1.5);

        % case 3
    elseif(Dis(1)<0 && abs(Dis(1))>27 && Dis(2)<27)
        disp("Moving the drone left");
        moveleft(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 4
    elseif(abs(Dis(1))<27 && Dis(2)>0 && abs(Dis(2))>27)
        disp("Moving the drone down");
        movedown(drone,'Distance',0.2,'Speed',1);
        pause(1.5);
   
        % case 5
    elseif(abs(Dis(1))<27 && Dis(2)<0 && abs(Dis(2))>27)
        disp("Moving the drone up");
        moveup(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 6
    elseif(Dis(1)>0 && abs(Dis(1))>27)
        disp("Moving right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 7
    elseif(Dis(1)<0 && abs(Dis(1))>27)
        disp("Moving left");
        moveleft(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % 나머지 경우 처리
    else
        disp("Hovering");

        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) < 11 || img(i,j,1) - img(i,j,3) > 0|| img(i,j,2) - img(i,j,3) > 20

                    img3(i,j,1) = 0;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;

                else

                    img3(i,j,1) = 122;
                    img3(i,j,2) = 48;
                    img3(i,j,3) = 160;
                    stage_pixel=stage_pixel+1;

                end
            end
        end
    end

     if (stage_pixel<200 && count_forward==1)
        break;
    end
end

disp("find_purple");
turn(drone, deg2rad(135));
moveleft(drone,'Distance',0.8);
stage_pixel=0;
count_forward=0;

moveforward(drone,'Distance',0.5);

% 3rd stage
while stage_pixel<200
    frame=snapshot(cam);
    img = double(frame);
    [R, C, X]=size(img);    
    img2=zeros(size(img));         % img2 변수를 사용하기 위해 사전할당을 통해 처리
    img3=zeros(size(img));         % img3 변수를 사용하기 위해 사전할당을 통해 처리
    for i =1:R
        for j=1:C
            if img(i,j,1) - img(i,j,2) > -5 || img(i,j,1) - img(i,j,3) > -5|| img(i,j,2) - img(i,j,3) > -40
                img2(i,j,1) = 255;
                img2(i,j,2) = 255;
                img2(i,j,3) = 255;

            else
                img2(i,j,:) = 0;
                img2(i,j,2) = 0;
                img2(i,j,3) = 0;
            end

        end
    end

    circle_ring = img2/255;
    circle_ring_Gray = rgb2gray(circle_ring);
    circle_ring_bi=imbinarize(circle_ring_Gray);
    bi2=imcomplement(circle_ring_bi);
    bw = bwareaopen(bi2,8000);
    bw = imcomplement(bw);
    se = strel('disk',10);
    bw2 = imclose(bw,se);
    bw3 = bwareaopen(bw2,8000);
    [B,L] = bwboundaries(bw3,'noholes');

    figure(1),imshow(bw3);
    axis on
    hold on

    for k = 1:length(B)
        boundary = B{k};
        plot(boundary(:,2),boundary(:,1),'w','LineWidth',2);
    end

    stats = regionprops(L,'Area','Centroid');

    threshold = 0.7;

    % loop over the boundaries
    for k = 1:length(B)

        % obtain (X,Y) boundary coordinates corresponding to label 'k'
        boundary = B{k};

        % compute a simple estimate of the object's perimeter
        delta_sq = diff(boundary).^2;
        perimeter = sum(sqrt(sum(delta_sq,2)));

        % obtain the area calculation corresponding to label 'k'
        area = stats(k).Area;

        % compute the roundness metric
        metric = 4*pi*area/perimeter^2;

        % display the results
        metric_string = sprintf('%2.2f',metric);

        % mark objects above the threshold with a black circle
        if metric > threshold
            area_meas=stats(k).Area;
            centroid = stats(k).Centroid;
            plot(centroid(1),centroid(2),'b');
        end

        text(boundary(1,2)-35,boundary(1,1)+13,metric_string,'Color','y',...
            'FontSize',14,'FontWeight','bold')

    end

    Dis=centroid-center_place;

    % case 1
    if(abs(Dis(1))<27 && abs(Dis(2))<27)
        disp("Moving the drone forward");
        if 19000<=area_meas && area_meas<23500
            disp("3.2m moveforward");
            moveforward(drone,'Distance',3.2,'Speed',0.6);
            count_forward = 1;
            
        elseif 23500<=area_meas && area_meas<28000
            disp("3.0m moveforward");
            moveforward(drone,'Distance',3.0,'Speed',0.6);
            count_forward = 1;
         
        elseif 28000<=area_meas && area_meas<34000
            disp("2.8m moveforward");
            moveforward(drone,'Distance',2.8,'Speed',0.6);
            count_forward = 1;
        
        elseif 34000<=area_meas && area_meas<39500
            disp("2.6m moveforward");
            moveforward(drone,'Distance',2.6,'Speed',0.6);
            count_forward = 1;
        
        elseif 39500<=area_meas && area_meas<51000
            disp("2.4m moveforward");
            moveforward(drone,'Distance',2.4,'Speed',0.6);
            count_forward = 1;
        
        elseif 51000<=area_meas && area_meas<66000
            disp("2.2m moveforward");
            moveforward(drone,'Distance',2.2,'Speed',0.6);
            count_forward = 1;
        
        elseif 66000<=area_meas && area_meas<95000
            disp("2.0m moveforward");
            moveforward(drone,'Distance',2,'Speed',0.6);
            count_forward = 1;

        elseif 95000<=area_meas && area_meas<145000
            disp("1.8m moveforward");
            moveforward(drone,'Distance',1.8,'Speed',0.6);
            count_forward = 1;


        elseif 145000<=area_meas && area_meas<220000
            disp("1.6m moveforward");
            moveforward(drone,'Distance',1.6,'Speed',0.6);
            count_forward = 1;


        elseif 220000<=area_meas && area_meas<360000
            disp("1.4m moveforward");
            moveforward(drone,'Distance',1.4,'Speed',0.6);
            count_forward = 1;

        elseif 360000<=area_meas
            disp("1.2m moveforward");
            moveforward(drone,'Distance',1.2,'Speed',0.6);
            count_forward = 1;

        else
            disp("3.4m moveforward");
            moveforward(drone,'Distance',3.4,'Speed',0.6);
            count_forward = 1;
        end

        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) <38 || img(i,j,1) - img(i,j,3) <10 || img(i,j,2)-img(i,j,3)>30

                    img3(i,j,1) = 0;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;

                else

                    img3(i,j,1) = 255;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;
                    stage_pixel=stage_pixel+1;

                end
            end
        end

        % case 2
    elseif(Dis(1)>0 && abs(Dis(1))>27 && Dis(2)<27)
        disp("Moving the drone right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 3
    elseif(Dis(1)<0 && abs(Dis(1))>27 && Dis(2)<27)
        disp("Moving the drone left");
        moveleft(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 4
    elseif(abs(Dis(1))<27 && Dis(2)>0 && abs(Dis(2))>27)
        disp("Moving the drone down");
        movedown(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 5
    elseif(abs(Dis(1))<27 && Dis(2)<0 && abs(Dis(2))>27)
        disp("Moving the drone up");
        moveup(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 6
    elseif(Dis(1)>0 && abs(Dis(1))>27)
        disp("Moving right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % case 7
    elseif(Dis(1)<0 && abs(Dis(1))>27)
        disp("Moving left");
        moveleft(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

        % 나머지 경우 처리
    else
        disp("Hovering");

        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) <38 || img(i,j,1) - img(i,j,3) <10 || img(i,j,2)-img(i,j,3)>30

                    img3(i,j,1) = 0;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;

                else

                    img3(i,j,1) = 255;
                    img3(i,j,2) = 0;
                    img3(i,j,3) = 0;
                    stage_pixel=stage_pixel+1;

                end
            end
        end
    end

    if (stage_pixel<200 && count_forward==1)
        break;
    end
end

disp("find_red");
land(drone);
