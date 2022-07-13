clc;
close all;
clear;

stage_pixel=0;
count_forward =0;
% 각 스테이지 별 원의 면적
area_meas=0;

center_place=[480,210];
centroid=zeros(size(center_place));

drone=ryze();
cam=camera(drone);
takeoff(drone);

moveup(drone,'Distance',0.5);


% 1st stage
while stage_pixel<200
    frame=snapshot(cam);
    img = double(frame);
    [R, C, X]=size(img);    
    for i =1:R
        for j=1:C
            if img(i,j,1) - img(i,j,2) > -5 || img(i,j,1) - img(i,j,3) > -5|| img(i,j,2) - img(i,j,3) > -50
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

    if (stage_pixel<200 && count_forward==1)
        break;
    end
end

if (stage_pixel<200 && count_forward==1)
    disp("notfound_green")
    turn(drone, deg2rad(90));
    moveup(drone,'Distance',0.6);
    moveforward(drone,'Distance',1.1,'Speed',0.8);
    stage_pixel=0;
    count_forward=0;

else 
    disp("find_green");
    turn(drone, deg2rad(90));
    moveup(drone,'Distance',0.6);
    moveforward(drone,'Distance',1.1,'Speed',0.8);
    stage_pixel=0;
    count_forward=0;
end

% 2nd round
while stage_pixel<200
    frame=snapshot(cam);
    img = double(frame);
    [R, C, X]=size(img);    
    for i =1:R
        for j=1:C
            if img(i,j,1) - img(i,j,2) > -5 || img(i,j,1) - img(i,j,3) > -5|| img(i,j,2) - img(i,j,3) > -50
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
            pause(1);

        elseif 27000<=area_meas && area_meas<32000
            disp("3.0m moveforward");
            moveforward(drone,'Distance',3.0,'Speed',0.8);
            count_forward = 1;
            pause(1);


        elseif 32000<=area_meas && area_meas<39000
            disp("2.8m moveforward");
            moveforward(drone,'Distance',2.8,'Speed',0.8);
            count_forward = 1;
            pause(1);


        elseif 39000<=area_meas && area_meas<47500
            disp("2.6m moveforward");
            moveforward(drone,'Distance',2.6,'Speed',0.8);
            count_forward = 1;
            pause(1);


        elseif 47500<=area_meas && area_meas<62000
            disp("2.4m moveforward");
            moveforward(drone,'Distance',2.4,'Speed',0.8);
            count_forward = 1;
            pause(1);


        elseif 62000<=area_meas && area_meas<84000
            disp("2.2m moveforward");
            moveforward(drone,'Distance',2.2,'Speed',0.8);
            count_forward = 1;
            pause(1);

        elseif 84000<=area_meas && area_meas<92000
            disp("2.0m moveforward");
            moveforward(drone,'Distance',2,'Speed',0.8);
            count_forward = 1;
            pause(1);

        elseif 92000<=area_meas && area_meas<140000
            disp("1.8m moveforward");
            moveforward(drone,'Distance',1.8,'Speed',0.8);
            count_forward = 1;
            pause(1);


        elseif 140000<=area_meas && area_meas<230000
            disp("1.6m moveforward");
            moveforward(drone,'Distance',1.6,'Speed',0.8);
            count_forward = 1;
            pause(1);


        elseif 230000<=area_meas && area_meas<380000
            disp("1.4m moveforward");
            moveforward(drone,'Distance',1.4,'Speed',0.8);
            count_forward = 1;
            pause(1);

        elseif 380000<=area_meas
            disp("1.2m moveforward");
            moveforward(drone,'Distance',1.2,'Speed',0.8);
            count_forward = 1;
            pause(1);

        else
            disp("3.4m moveforward");
            moveforward(drone,'Distance',3.4,'Speed',0.8);
            count_forward = 1;
            pause(1);
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

    elseif(Dis(1)>0 && abs(Dis(1))>27)
        disp("Moving right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);

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

if (stage_pixel<200 && count_forward==1)

     disp("notfound_purple");
    turn(drone, deg2rad(135));
    moveleft(drone,'Distance',0.8);
    stage_pixel=0;
    count_forward=0;

else 
    disp("find_purple");
    turn(drone, deg2rad(135));
    moveleft(drone,'Distance',0.8);
    stage_pixel=0;
    count_forward=0;
end



% 3rd stage
while stage_pixel<200
    frame=snapshot(cam);
    img = double(frame);
    [R, C, X]=size(img);    
    for i =1:R
        for j=1:C
            if img(i,j,1) - img(i,j,2) > -5 || img(i,j,1) - img(i,j,3) > -5|| img(i,j,2) - img(i,j,3) > -50
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
            pause(3);

        elseif 23500<=area_meas && area_meas<28000
            disp("3.0m moveforward");
            moveforward(drone,'Distance',3.0,'Speed',0.6);
            pause(3);
    

        elseif 28000<=area_meas && area_meas<34000
            disp("2.8m moveforward");
            moveforward(drone,'Distance',2.8,'Speed',0.6);
            pause(3);


        elseif 34000<=area_meas && area_meas<39500
            disp("2.6m moveforward");
            moveforward(drone,'Distance',2.6,'Speed',0.6);
            pause(3);


        elseif 39500<=area_meas && area_meas<51000
            disp("2.4m moveforward");
            moveforward(drone,'Distance',2.4,'Speed',0.6);
            pause(3);


        elseif 51000<=area_meas && area_meas<66000
            disp("2.2m moveforward");
            moveforward(drone,'Distance',2.2,'Speed',0.6);
            pause(3);

        elseif 66000<=area_meas && area_meas<95000
            disp("2.0m moveforward");
            moveforward(drone,'Distance',2,'Speed',0.6);
            pause(3);

        elseif 95000<=area_meas && area_meas<145000
            disp("1.8m moveforward");
            moveforward(drone,'Distance',1.8,'Speed',0.6);
            pause(3);


        elseif 145000<=area_meas && area_meas<220000
            disp("1.6m moveforward");
            moveforward(drone,'Distance',1.6,'Speed',0.6);
            pause(3);


        elseif 220000<=area_meas && area_meas<360000
            disp("1.4m moveforward");
            moveforward(drone,'Distance',1.4,'Speed',0.6);
            pause(3);

        elseif 360000<=area_meas
            disp("1.2m moveforward");
            moveforward(drone,'Distance',1.2,'Speed',0.6);
            pause(3);

        else
            disp("3.4m moveforward");
            moveforward(drone,'Distance',3.4,'Speed',0.6);
            pause(3);
        end

        frame=snapshot(cam);
        img = double(frame);
        [R, C, X]=size(img);        

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) <38 || img(i,j,1) - img(i,j,3) <10 || img(i,j,2)-img(i,j,3)>20

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
    elseif(Dis(1)>0 && abs(Dis(1))>27)
        disp("Moving right");
        moveright(drone,'Distance',0.2,'Speed',1);
        pause(1.5);
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
                if img(i,j,1) - img(i,j,2) <38 || img(i,j,1) - img(i,j,3) <10 || img(i,j,2)-img(i,j,3)>20

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
end

disp("find_red");
land(drone);
