2022 미니드론 자율주행 경진대회 아주위잉위잉 팀 코드 설명서
===================================
##### 대한전기학회에서 주최한 2022 미니드론 자율주행 경진대회에 참가한 아주위잉위잉 팀의 코드 설명문입니다.

## 목차
#### 1. Tello 드론 소개
#### 2. 대회 진행 전략
#### 3. 알고리즘 설명 및 소스코드 구현
##### 1) 원의 중심 찾기
##### 2) 주행
##### 3) 표식의 색 인식
#### 4. 코드 실행 결과
#### 5. 발생할 수 있는 문제점
#### 6. 팀원 소개

# 1.Tello 드론 소개
![image](https://user-images.githubusercontent.com/92336598/178506652-6db579d3-0449-4e45-84bd-9978b4160c35.png)

위 사양과 같은 [ryze Tello](https://www.ryzerobotics.com/kr/tello/specs "텔로") 모델을 사용하였습니다.

드론 제어에 사용된 코드 설계에 있어 [MATLAB Tello Support 패키지](https://kr.mathworks.com/hardware-support/tello-drone-matlab.html)를 사용했습니다.

# 2. 대회 진행 전략
<img width="723" alt="주행전략" src="https://user-images.githubusercontent.com/103809007/178740334-c7d0e441-766a-4d5d-91dd-1b1ce4285835.PNG">

위와 같은 주행 전략을 정리하면 다음과 같습니다.

1). 출발(takeoff) 후 파란색 영역을 식별하고 원모양을 구별해내고 원의 중심을 찾습니다.

2). 드론이 원의 중심으로 이동한 후 현재 드론이 촬영하고 있는 원의 영역 크기에 따른 거리 만큼 전진합니다.

3). 초록색 표식을 인식하면 90도 우회전하고 일정거리 상승 or 전진합니다.

4). 위의 과정 1,2 번을 반복하여 stage 2에서 원을 식별하여 전진합니다.

5). 보라색 표식을 인식 후 약 135도 정도 회전하고 왼쪽으로 이동, 전진합니다.

6). 위의 과정 1,2 번을 반복하여 stage3에서 원을 식별하여 전진합니다.

7). 빨간색 표식을 인식 후 착륙(land)합니다. 

대회 준비를 위해 경기장과 유사한 트랙을 아래의 사진과 같이 만들어서 주행 연습을 진행했습니다.

![image](https://user-images.githubusercontent.com/92336598/178759491-acc394af-033b-4af6-89cd-c3dc03f1c821.png)

# 3. 알고리즘 설명 및 소스코드 구현
### 1) 원의 중심 찾기
![image](https://user-images.githubusercontent.com/103806351/178755274-7e65de11-013a-48c4-b558-f40d027a5249.png)


 저희는 링의 중심을 찾기 위하여 우선 크로마키 천이 파란색이라는 것을 이용하였습니다. 

우선 드론의 카메라를 통해서 받아온 double형의 frame 이미지에서 파란색만 추출하는 과정을 진행하였습니다. 

frame 이미지의 RGB 색상 중에서 R과 G의 차이, R과 B의 차이, G와 B의 차이를 기준으로 파란색을 판단하였습니다. 

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

    
위와 같이 RGB 값을 이용하여 파란색인지 판단을 한 후, 파란색인 부분은 흰색으로, 그 외의 색깔들은 검은색으로 변환하는 처리 과정을 진행하였습니다.
 
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
    
파란색 추출 이미지 처리를 통해 얻은 RGB 이미지인 circle_ring을 회색조로 변환한 후, 이를 이진화하여 이진 영상으로 변환하고 보수를 사용하여 색을 반전시켜주었습니다. 
    
원을 정확하게 찾기 위해 영상 필터링 및 향상 과정으로 bwareaopen을 사용하여 8000픽셀 이하의 객체를 모두 지웠고, 이를 반전시켜서 strel을 통해서 모폴로지 연산을 하여 
    
이미지를 원형구조로 만들어 주었습니다. imclose로 모폴로지 닫기 연산 수행하고, 다시 bwareaopen을 이용하여 8000픽셀 이하의 객체들을 지워주었습니다.
    
bwboundaries가 내부 윤곽선을 찾는 것을 방지하기 위해 noholes 옵션을 지정하여 외부 경계선에만 초점두게 하였습니다. 
    
위의 과정들을 통해서 아래의 이미지를 얻을 수 있었습니다.
    
![image](https://user-images.githubusercontent.com/103806351/178756355-5e85ce52-5cf8-4d42-ac08-3f5ac4d1dc1d.png)

다음으로는 위에서 얻은 이미지에서 원을 찾고 원의 중심을 찾는 과정입니다.
    
    % 원의 경계를 하얀색 선으로 plot
    for k = 1:length(B)
        boundary = B{k};
        plot(boundary(:,2),boundary(:,1),'w','LineWidth',2);
    end
    
위의 과정을 통해서 레이블 행렬을 표시하고 각 경계를 그려줍니다. 다음으로 이미지에서 표시된 각 객체들의 면적과 둘레를 측정하고 이를 기반으로 
    
객체의 원형률을 나타내는 메트릭을 만들었습니다.
  
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
    
메트릭의 값이 1일때가 원이고, 1에 가까울수록 원형에 가깝다는 것을 의미합니다. 
    
 따라서, 메트릭에 대한 특정 임계값을 ‘threshold = 0.7’이라고 지정하여 객체를 원으로 판단하는 기준을 마련하였습니다.
    
        % display the results
        metric_string = sprintf('%2.2f',metric);

        % 기준 수치인 threshold보다 클 경우 아래의 명령을 수행
        if metric > threshold
            area_meas=stats(k).Area;        % 해당 영역의 면적을 area_meas에 저장
            centroid = stats(k).Centroid;   % 해당 영역의 중점을 centroid에 저장
            plot(centroid(1),centroid(2),'r');  % centroid(중점)을 figure 1에 plot 
        end
        
위와 같이 객체의 메트릭이 threshold = 0.7보다 큰 경우를 원으로 판단하고, Centroid를 통해서 원의 중심점을 찾고 ‘centroid’ 변수에 원의 중심 좌표를 저장하였습니다.

![image](https://user-images.githubusercontent.com/103806351/178756103-75f07001-9bfa-4814-92a7-d1791d88d509.png)

위의 그림이 원 판단 과정까지 거친 최종 이미지입니다. 크로마키 천의 원형 링 객체가 0.9의 메트릭의 값으로 원으로 판단되었습니다.

원의 중심 좌표도 아래의 이미지와 같이 알맞게 저장되었음을 확인 할 수 있었습니다.

![image](https://user-images.githubusercontent.com/103806351/178755786-f52fb225-30a9-41d9-9e4d-e482f3d702ad.png)

![image](https://user-images.githubusercontent.com/103806351/178755639-7ee8f386-ba88-48e8-a681-d8720d42e555.png)![image](https://user-images.githubusercontent.com/103806351/178755655-af3ac6fe-169e-40a2-a5da-0a73617e99d1.png)

이미지 처리 전, 후 비교

### 2) 주행
다음과 같이 직진 여부를 결정하기 위해 현재 원의 중점 좌표인 centroid와 기준이 되는 중점의 위치 center_place의 위치 차이를 Dis라는 새로운 변수를 도입하여 저장합니다.

    % 3-1) 원의 중심 찾기 알고리즘
    Dis=centroid-center_place;  % 현재 원의 위치에서 기준이 되는 중점의 위치의 차이를 Dis로 저장

그다음, Dis의 값에 따라 case를 나눠 각 case에 따라 다음과 같이 움직이도록 코드를 설계했습니다.

![image](https://user-images.githubusercontent.com/92336598/178763354-bd2ee51f-74a8-4100-bac0-8d8ac2f5cb95.png)

case 1은 기준 영역 내에 들어온 경우로 각 거리에 따른 원의 넓이를 실험적으로 측정하였고 그 값에 따른 드론의 직진 거리를 다음과 같이 설정하였습니다. 그다음 해당되는 거리를 한 번에 이동하도록 하였고, 이동한 횟수인 count_forward를 1로 설정해 그 뒤, moveforward를 하지 못하도록 코드를 설계하였습니다.

    % case 1
    if(abs(Dis(1))<27 && abs(Dis(2))<27)    % x 좌표 차이, y 좌표 차이가 27보다 작을 경우 앞으로 전진
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
        
전진 후, 그 앞에는 표식이 보이고 이미지를 다시 읽어 각 표식의 픽셀값을 읽어오도록 코드를 설계했습니다.

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
        
그 외의 case2~case7까지의 드론 제어 설계는 위의 사진과 같은 방식으로 설계하였고 그 코드는 다음과 같습니다.

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
        % 이미지를 읽어오는 
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
    end

1회 전진했을 경우 count_forward=1이므로 더이상 전진하지 않고 다음 loop를 수행하도록 하기 위해 아래와 같은 코드를 설계했습니다.

    % 전진한 횟수가 1회인 경우 해당 while loop를 빠져나오도록 설정
    if (stage_pixel<200 && count_forward==1)
        break;
    end
    
1st stage를 지나게 되면, 2nd stage에 들어서게 되고 이 역시 위와 같은 방법으로 코드를 설계하였습니다.
마찬가지로, 2nd stage를 지나게 되면 3rd stage에 들어서게 되고 같은 방법으로 코드를 설계하였습니다.

### 3) 표식의 색 인식
R, G, B 각 색에 따른 측정되는 세기의 정도가 다르기에, 이를 실험적으로 드론이 촬영한 이미지의 각 3가지 영역(R,G,B)값들의 차이를 측정해 값들을 설정했습니다.

드론이 받아온 이미지에서 img(i,j,1)은 이미지의 R영역이고, img(i,j,2)은 이미지의 G영역, img(i,j,3)은 이미지의 B영역을 뜻합니다. 

#### Red
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

위의 코드는 빨간색을 식별하는 코드입니다. 실험적으로 측정한 결과 빨간색과 그 외의 색들을 구분하는 임계값은 다음과 같습니다.

R과 G의 차이가 <38일때, R과 B의 차이가 <10일때, G와 B의 차이가 > 30 일때 입니다. 

if에서 OR 구문을 사용하여 이 범위 내에 들면 드론이 촬영한 이미지의 각 3가지 영역을 모두 0처리(검은색)합니다. 즉 빨간색을 식별하지 못한 것입니다.

else 구문에서는 임계값들의 범위 내에 모두 들지 않으면 추출하고자 하는 빨간색을 R=255, G=0, B=0로 빨간색으로 나타냅니다.

따라서 이 경우는 빨간색을 인식한 경우라고 볼 수 있습니다.

빨간색을 인식한 경우 stage_pixel값을 빨간색이 인식될때마다 1씩 증가시켜 빨간색 이미지 pixel 크기를 stage_pixel에 저장시킵니다.

마지막으로 드론이 빨간색 표식을 인식하여 드론이 착륙(land)하면 stage_pixel을 다시 0으로 초기화시킵니다.

드론이 빨간색을 인식하였을 경우 다음과 같이 이미지를 인식하게 됩니다.

![image](https://user-images.githubusercontent.com/103809007/178746987-f3ad0410-bc02-4fe5-af0d-45136cedf64b.png)


#### Green

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

위의 코드는 초록색을 식별하는 코드입니다. 실험적으로 측정한 결과 빨간색과 그 외의 색들을 구분하는 임계값은 다음과 같습니다.

R과 G의 차이가 >20일때, R과 B의 차이가 >10일때, G와 B의 차이가 < 15 일때 입니다. 

if에서 OR 구문을 사용하여 이 범위 내에 들면 드론이 촬영한 이미지의 각 3가지 영역을 모두 0처리(검은색)합니다. 즉 초록색을 식별하지 못한 것입니다.

else 구문에서는 임계값들의 범위 내에 모두 들지 않으면 추출하고자 하는 초록색을 R=0, G=255, B=0 로 초록색으로 나타냅니다.

따라서 이 경우는 초록색을 인식한 경우라고 볼 수 있습니다.

초록색을 인식한 경우 stage_pixel값을 초록색이 인식될때마다 1씩 증가시켜 초록색 이미지 pixel 크기를 stage_pixel에 저장시킵니다.

stage 1에서 드론이 초록색 표식을 인식하여 드론이 90도 우회전 하면 stage_pixel을 다시 0으로 초기화시킵니다.

드론이 초록색을 인식하였을 경우 다음과 같이 이미지를 인식하게 됩니다.

![image](https://user-images.githubusercontent.com/103809007/178748671-e6a45854-8c3c-40bb-8980-14139eedc9f0.png)


#### Purple

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
    
위의 코드는 보라색을 식별하는 코드입니다. 실험적으로 측정한 결과 빨간색과 그 외의 색들을 구분하는 임계값은 다음과 같습니다.

R과 G의 차이가 <11일때, R과 B의 차이가 >0일때, G와 B의 차이가 > 20 일때 입니다. 

if에서 OR 구문을 사용하여 이 범위 내에 들면 드론이 촬영한 이미지의 각 3가지 영역을 모두 0처리(검은색)합니다. 즉 보라색을 식별하지 못한 것입니다.

else 구문에서는 임계값들의 범위 내에 모두 들지 않으면 추출하고자 하는 색을 R=122, G=48, B=160 로 보라색으로 나타냅니다.

따라서 이 경우는 보라색을 인식한 경우라고 볼 수 있습니다.

보라색을 인식한 경우 stage_pixel값을 초록색이 인식될때마다 1씩 증가시켜 초록색 이미지 pixel 크기를 stage_pixel에 저장시킵니다.

stage 2에서 드론이 보라색 표식을 인식하여 드론이 약135도 우회전 하면 stage_pixel을 다시 0으로 초기화시킵니다.

드론이 보라색을 인식하였을 경우 다음과 같이 이미지를 인식하게 됩니다.

![image](https://user-images.githubusercontent.com/103809007/178750574-c0969eff-2182-4d28-ab6b-56776c9ca1a4.png)

# 4. 코드 실행 결과

성공적으로 마지막 단계인 3단계까지 드론이 이동해 착륙하는 것을 확인할 수 있었습니다.

# 5. 발생할 수 있는 문제점

1. 색 식별시 표식과 비슷한 계열의 색의 배경이 드론 카메라에 찍힐 경우 배경을 표식으로 인식할 수 있어서 정확한 제어를 하지 못하는 경우가 발생할 수 있습니다. 따라서 빛의 세기등을 고려하여 색의 임계값을 까다롭게 설정해야합니다. 임계값을 까다롭게 설정할 경우 표식 색 인식의 정확도가 떨어질 수 있어서 제대로 표식을 인식하지 못하는 문제가 발생할 수 있습니다.

2. 드론이 주행 중 원의 픽셀을 조금이라도 발견하지 못하면 정상적인 주행을 할 수 없게 됩니다.

3. stage3에서 보라색 표식 인식 후 약 135도를 우회전하게 되는데 실제 맵에서는 120도~150도 사이값이기 때문에 거리와 각도에 따라 원을 식별하지 못하는 경우가 발생할 수 있습니다.

4. 주변 배경에 파란색 계열의 원 모양이 존재 할 경우 이를 인식하여 이 물체에 대한 원의 중심을 찾게 되기 때문에 정상적인 주행이 불가능 할 수 있습니다.

# 6. 팀원 소개
### 아주대학교 전자공학과 네트로닉스 소학회 소속
|직책|이름|
|---|---|
|팀장|석대근|
|팀원1|장재형|
|팀원2|이민성|
