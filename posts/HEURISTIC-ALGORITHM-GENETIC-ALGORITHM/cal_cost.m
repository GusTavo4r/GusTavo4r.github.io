function cost = cal_cost(my_answer)

% 标准样本原始数据读入
input=dlmread('dataform2018.csv');
[m,n]=size(input);
Nsample=m/2; %样本数量
Npoint=n;%温度点数
volt=zeros(Nsample,Npoint);%电压
tempr=zeros(Nsample,Npoint);%温度
est_tempr=zeros(Nsample,Npoint);%温度估测值
for p=1:Nsample
    volt(p,:)=input(2*p,:);
    tempr(p,:)=input(2*p-1,:);
end

% 定标计算
     for j=1:Nsample
         selecet_volt=volt(j,my_answer);%标定点电压
         select_tempr=tempr(j,my_answer);%标定点温度
         est_tempr(j,:)=interp1(selecet_volt,select_tempr,volt(j,:),'spline');%三次样条插值
     end
     
     % 成本计算
     Q=50;
     error=abs(tempr-est_tempr);%单点误差的绝对值

     less0_5=(error<=0.5);
     less1_0=(error<=1.0);
     less1_5=(error<=1.5);
     less2_0=(error<=2);
     over2_0=(error>2);

     sij=1*(less1_0-less0_5)+4*(less1_5-less1_0)+10*(less2_0-less1_5)+10000*over2_0;%单点误差成本
     si=sum(sij,2)+Q*ones(Nsample,1)*length(my_answer);%单个样本个体标定误差
     cost=sum(si)/Nsample;%方案成本
end