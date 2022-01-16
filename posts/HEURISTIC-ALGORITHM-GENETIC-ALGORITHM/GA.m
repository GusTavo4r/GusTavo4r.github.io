fitvalue=[4 3 2 1]; %奖项对应的适应度值
totalf=sum(fitvalue); %适应值之和
p=fitvalue./totalf; %单个个体被选中的概率
q=cumsum(p); %每个个体的累积概率
c1=0; %用于存放编号1被选中的次数
c2=0; %用于存放编号2被选中的次数
c3=0; %用于存放编号3被选中的次数
c4=0; %用于存放编号4被选中的次数
while c1+c2+c3+c4<=996
    fitin=1;
    newin=1;
    m=sort(rand(4,1)); %生成一组从小到大排列的随机数组
    while newin<=4
        if q(fitin)>m(newin)
            switch fitin
                case 1
                    c1=c1+1;
                case 2
                    c2=c2+1;
                case 3
                    c3=c3+1;
                case 4
                    c4=c4+1;
            end
            newin=newin+1;
        else
            fitin=fitin+1;
        end
    end
end
disp('***************************************')
disp(['抽中“参与奖”的次数为：',num2str(c1)]);
disp(['抽中“参与奖”的概率为：',num2str(c1/(c1+c2+c3+c4))]);
disp('***************************************')
disp(['抽中“三等奖”的次数为：',num2str(c2)]);
disp(['抽中“三等奖”的概率为：',num2str(c2/(c1+c2+c3+c4))]);
disp('***************************************')
disp(['抽中“二等奖”的次数为：',num2str(c3)]);
disp(['抽中“二等奖”的概率为：',num2str(c3/(c1+c2+c3+c4))]);
disp('***************************************')
disp(['抽中“一等奖”的次数为：',num2str(c4)]);
disp(['抽中“一等奖”的概率为：',num2str(c4/(c1+c2+c3+c4))]);
disp('***************************************')