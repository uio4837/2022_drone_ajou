
sum_JPY=0;
sum_JPY=sum_JPY+floor(JPY/10000);
a=rem(JPY,10000);
sum_JPY=sum_JPY+floor(a/5000);
b=rem(a,5000);
sum_JPY=sum_JPY+floor(b/2000);
c=rem(b,2000);
sum_JPY=sum_JPY+floor(c/1000)

sum_CNY=0;
sum_CNY=sum_JPY+floor(CNY/100);
a=rem(CNY,100);
sum_CNY=sum_CNY+floor(a/50);
b=rem(a,50);
sum_CNY=sum_CNY+floor(b/20);
c=rem(b,20);
sum_CNY=sum_CNY+floor(c/5);
d=rem(c,5);
sum_CNY=sum_CNY+floor(d/1)

fprintf("원화:%d장 달러:%d장 유로%d장 엔화:%d장 위안화:%d장",sum_KRW,sum_USD,sum_EUR,sum_JPY,sum_CNY)