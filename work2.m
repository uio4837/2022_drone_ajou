sum_KRW=0;
sum_KRW=sum_KRW+floor(KRW/50000);
a=rem(KRW,50000);
sum_KRW=sum_KRW+floor(a/10000)
b=rem(a,10000);
sum_KRW=sum_KRW+floor(b/5000);
c=rem(b,5000);
sum_KRW=sum_KRW+floor(c/1000);

sum_USD=0;
sum_USD=sum_USD+floor(USD/100);
a=rem(USD,100);
sum_USD=sum_USD+floor(a/50);
b=rem(a,50);
sum_USD=sum_USD+floor(b/20);
c=rem(b,20);
sum_USD=sum_USD+floor(c/10);
d=rem(c,10);
sum_USD=sum_USD+floor(d/5);
e=rem(d,5);
sum_USD=sum_USD+floor(e/2);
f=rem(e,2);
sum_USD=sum_USD+floor(f/1)

sum_EUR=0;
sum_EUR=sum_EUR+floor(EUR/500);
a=rem(EUR,500);
sum_EUR=sum_EUR+floor(a/200);
b=rem(a,200);
sum_EUR=sum_EUR+floor(b/100);
c=rem(b,100);
sum_EUR=sum_EUR+floor(c/50);
d=rem(c,50);
sum_EUR=sum_EUR+floor(d/20);
e=rem(d,20);
sum_EUR=sum_EUR+floor(e/10);
f=rem(e,10);
sum_EUR=sum_EUR+floor(f/5)

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
