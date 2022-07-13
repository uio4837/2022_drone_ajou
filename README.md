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

[ryze Tello](https://www.ryzerobotics.com/kr/tello/specs "텔로") 모델을 사용하였습니다.

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

# 3. 알고리즘 설명 및 소스코드 구현
### 1) 원의 중심 찾기
크로마키 천이 파란색이라는 환경을 이용하기 위하여, 드론의 카메라를 통해서 받아온 double형의 frame 이미지에서 다음의 과정을 통해 파란색 부분만을 추출한다.

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

파란색만 남은 이미지를 circle_ring = img2/255; 에 출력값으로 받아온다.

 circle_ring_Gray = rgb2gray(circle_ring);
    circle_ring_bi=imbinarize(circle_ring_Gray);
    bi2=imcomplement(circle_ring_bi);
    bw = bwareaopen(bi2,8000);
    bw = imcomplement(bw);
    se = strel('disk',10);
    bw2 = imclose(bw,se);
    bw3 = bwareaopen(bw2,8000);
    [B,L] = bwboundaries(bw3,'noholes');

이 과정들을 통해서 이미지를 회색으로 변환 후, 이진 영상의 형태로 바꾸어 불필요한 픽셀을 제거하고 구멍을 채워서 이미지를 깨끗하게 처리하여 준다.

### 2) 주행
원의 넓이를 실험적으로 측정하였고, 그 값에 따른 드론의 직진 거리를 다음과 같이 설정하였습니다.

        if 50000<=area_meas && area_meas<60000
            disp("3.2m moveforward");
            moveforward(drone,'Distance',3.2,'Speed',0.8);

        elseif 60000<=area_meas && area_meas<74000
            disp("3.0m moveforward");
            moveforward(drone,'Distance',3.0,'Speed',0.8);


        elseif 74000<=area_meas && area_meas<85000
            disp("2.8m moveforward");
            moveforward(drone,'Distance',2.8,'Speed',0.8);


        elseif 85000<=area_meas && area_meas<105000
            disp("2.6m moveforward");
            moveforward(drone,'Distance',2.6,'Speed',0.8);


        elseif 105000<=area_meas && area_meas<130000
            disp("2.4m moveforward");
            moveforward(drone,'Distance',2.4,'Speed',0.8);


        elseif 130000<=area_meas && area_meas<165000
            disp("2.2m moveforward");
            moveforward(drone,'Distance',2.2,'Speed',0.8);


        elseif 160000<=area_meas && area_meas<220000
            disp("2.0m moveforward");
            moveforward(drone,'Distance',2,'Speed',0.8);


        elseif 220000<=area_meas && area_meas<360000
            disp("1.8m moveforward");
            moveforward(drone,'Distance',1.8,'Speed',0.8);


        elseif 360000<=area_meas && area_meas<460000
            disp("1.6m moveforward");
            moveforward(drone,'Distance',1.6,'Speed',0.8);


        elseif 460000<=area_meas && area_meas<600000
            disp("1.4m moveforward");
            moveforward(drone,'Distance',1.4,'Speed',0.8);


        elseif 600000<=area_meas
            disp("1.2m moveforward");
            moveforward(drone,'Distance',1.2,'Speed',0.8);

        else
            disp("3.4m moveforward");
            moveforward(drone,'Distance',3.4,'Speed',0.8);
        end
        
### 3) 표식의 색 인식
R, G, B 각 색에 따른 측정되는 세기의 정도가 다르기에, 이를 실험적으로 드론이 촬영한 이미지의 각 3가지 영역(R,G,B)값들의 차이를 측정해 값들을 설정했습니다.

드론이 받아온 이미지에서 img(i,j,1)은 이미지의 R영역이고, img(i,j,2)은 이미지의 G영역, img(i,j,3)은 이미지의 B영역을 뜻합니다. 

#### Red
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

위의 코드는 빨간색을 식별하는 코드입니다. 실험적으로 측정한 결과 빨간색과 그 외의 색들을 구분하는 임계값은 다음과 같습니다.

R과 G의 차이가 <38일때, R과 B의 차이가 <10일때, G와 B의 차이가 > 20 일때 입니다. 

if에서 OR 구문을 사용하여 이 범위 내에 들면 드론이 촬영한 이미지의 각 3가지 영역을 모두 0처리(검은색)합니다. 즉 빨간색을 식별하지 못한 것입니다.

else 구문에서는 임계값들의 범위 내에 모두 들지 않으면 추출하고자 하는 빨간색을 R=255로 빨간색으로 나타내고 이를 제외한 나머지 색들을 G=0, B=0 으로 설정하여 같이 검은색 처리합니다.

따라서 이 경우는 빨간색을 인식한 경우라고 볼 수 있습니다.

빨간색을 인식한 경우 stage_pixel값을 빨간색이 인식될때마다 1씩 증가시켜 빨간색 이미지 pixel 크기를 stage_pixel에 저장시킵니다.

마지막으로 드론이 빨간색 표식을 인식하여 드론이 착륙(land)하면 stage_pixel을 다시 0으로 초기화시킵니다.

드론이 빨간색을 인식하였을 경우 다음과 같이 이미지를 인식하게 됩니다.

![image](https://user-images.githubusercontent.com/103809007/178746987-f3ad0410-bc02-4fe5-af0d-45136cedf64b.png)


#### Green

        for i =1:R
            for j=1:C
                if img(i,j,1) - img(i,j,2) > 20 || img(i,j,1) - img(i,j,3) > 10|| img(i,j,2) - img(i,j,3) < 29

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

# 4. 코드 실행 결과
성공적으로 마지막 단계인 3단계까지 드론이 이동해 착륙하는 것을 확인할 수 있었습니다.

# 5. 발생할 수 있는 문제점

# 6. 팀원 소개
### 아주대학교 전자공학과 네트로닉스 소학회 소속
|직책|이름|
|---|---|
|팀장|석대근|
|팀원1|장재형|
|팀원2|이민성|
