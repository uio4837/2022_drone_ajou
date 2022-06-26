KRW=sym ('KRW');
KRW=input("원화를 입력하세요 : ")
USD=KRW/1229.50; %2022.04.16 기준 환율 적용
EUR=KRW/1328.84;
JPY=KRW/9.72;
CNY=KRW/192.62;
fprintf("USD: %f$ EUR: %f€ JPY: %f￥ CNY: %f元",USD,EUR,JPY,CNY)
