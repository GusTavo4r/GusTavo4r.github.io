function main()
clear 
clc
popsize = 50; %种群大小
chromlength = 90; %二进制编码长度
pm = 0.8; %变异率 
pc = 0.9; %交叉率
pi = 0.01; %倒位率
pop = zeros( popsize, chromlength); %种群初始化
for p=1:popsize
    for n=1:chromlength    
        if rand < 0.08    %个体每个编码有8%的可能为1
        pop(p,n)=1;
        end
    end
end

            
itertime = 100; %迭代次数

y = zeros( itertime,1); % 每代最大适应度
fitvalue = cal_fitvalue( pop ); %计算初始种群每个个体的适应度
[bestindividual, bestfit] = best( pop, fitvalue); %寻找初始种群最优个体和适应度
y(1) = bestfit;
besti=bestindividual; % 历史最优个体
bestf=bestfit; % 历史最大适应度

for i = 1:itertime
    newpop = selection( pop, fitvalue ); %选择
    newpop = crossover( newpop, pc ); %交叉
    newpop = mutation( newpop, pm ); %变异
    newpop = inversion( newpop, pi );%倒位
    pop = newpop; %更新种群
    fitvalue = cal_fitvalue( pop ); %计算适应度
    [bestindividual, bestfit] = best( pop, fitvalue); %寻找最优个体和适应度
    
    %保留历史最优个体
    if bestf>bestfit    
        pop(1,:)=besti;  %不在本代种群的历史最优个体取代本代第一个个体
        fitvalue(1)=bestf;
    else   %更新历史最优个体、历史最大适应度
        bestf=bestfit;
        besti=bestindividual;
    end
    
    y(i) = bestf;    
end

%作图：种群最大适应度随迭代次数的变化图像
x = 1:itertime;
plot( x, y, '*');
title('种群最大适应度随迭代次数的变化趋势' ); 
xlabel('迭代次数');
ylabel('种群最大适应度');


%输出最优解   
best_answer = find( besti == 1 ) ;  %最优方案的温度测定点序号
fprintf( '最优方案标定%d个温度：\n',length(best_answer));
fprintf( '%d ',best_answer - 21);

%输出最低成本
best_cost = cal_cost(best_answer);
fprintf( '\n最低的成本为%5.2f\n', best_cost);
end