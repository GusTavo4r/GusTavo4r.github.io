# 启发式算法-人工蜂群算法（ABC）


## 1.简述[^1]

人工蜂群算法是模仿蜜蜂行为提出的一种优化方法，是集群智能思想的一个具体应用，它的主要特点是不需要了解问题的特殊信息，只需要对问题进行优劣的比较，通过各人工蜂个体的局部寻优行为，最终在群体中使全局最优值突现出来，有着较快的收敛速度。为了解决多变量函数优化问题，Karaboga提出了人工蜂群算法`ABC`模型(artificial bee colony algorithm)。 人工蜂群算法受启发于蜜蜂的寻蜜和采蜜过程，相比于常见的启发式算法，它的优点在于其使用了较少的控制参数，并且鲁棒性强，在每次迭代过程中都会进行全局和局部的最优解搜索，因此能够找到最优解的概率大大增加。 

相比于遗传算法来说，人工蜂群算法在局部的收敛和寻优能力上要更为出色，不会出现遗传算法的“早熟”现象，并且算法的复杂度也较低。但由于遗传算法有交叉以及变异的操作，因此遗传算法在全局最优值的搜索上要优于人工蜂群算法。此外，人工蜂群算法适用于进行连续函数的全局优化问题，而不适用于一些离散函数。 

## 2.算法过程[^2][^3]

ABC算法中的蜜蜂分为３类：雇佣蜂(Leader)、观察蜂(Follower)和侦察蜂(Scouter)。对于给定的问题，每个「食物源」均是搜索空间中的「候选解」。食物源的优劣即可行解的好坏是用蜜源花蜜量的大小即「适应度」来评价的。 在种群的进化过程中，不同种类的蜜蜂使用不同的操作方式来搜索空间。

雇佣蜂：雇佣蜂即为「采蜜蜂」与食物源的位置相对应，一个食物源对应一个采蜜蜂。在人工蜂群算法中，食物源的个数与引领蜂的个数相等；采蜜蜂发现食物源信息并以一定的概率招募观察蜂；概率根据适应度值以「轮盘赌」的方法计算。 

（1）设待优化问题的维数为 $$D$$ ，用 $$NP$$ 表示群体规模，食物源的数量（等于雇佣蜂数量）也即解的数量为 $$SP$$  ，第 $$i$$ 个解为 $$R=(R_{i1},R_{i2},\cdots,R_{iD})$$ ，算法开始运行时，首先初始化这 $$SP$$ 个候选解的位置：

$$
R_{ij}=L_j+rand(0,1)\times(U_j-L_j)\tag{1}
$$



其中 $$i=1,2,\cdots,SP$$ 为第$$i$$个食物来源(解)； $$j=1,2,\cdots,D$$ 为第$$j$$个维度； $$U$$ 和 $$L$$ 为搜索空间的上限和下限； $$rand$$ 生成随机数。

（2）此时蜂群中只有雇佣蜂(引领蜂)和观察蜂，且数量分别为 $$SP$$ 和 $$(NP-SP)$$ 。首先由雇佣蜂对蜜源位置进行更新，对第 $$j$$ 维度第 $$i$$ 个食物源位置的更新方法：


$$
R^*_{ij}=R_{ij}+rand(-1,1)\times(R_{ij}-R_{rj})\tag{2}
$$


其中 $$r\neq i$$ 即 $$r=1,2,\cdots,i-1,i+1,\cdots,SP$$ ， $$R_{rj}$$ 表示除当前解 $$R_{ij}$$ 以外的任一食物源(解)。产生新解 $$R_{ij}^*$$ 后，由「贪婪选择」在 $$R_{ij}$$ 和 $$R_{ij}^*$$ 之间进行选择。并且对每个食物源计算其对应的目标值 $$f_i$$ 和适应值 $$fit_i$$ ，以最小化问题为例，适应度由下式计算：


$$
fit_i=\begin{cases}1\over1+f_i &f_i\gt0\\1+|f_i|&f_i\leqslant0\end{cases}\tag{3}
$$


另外 $$f_i$$ 即为目标函数值。贪婪选择由下式进行：


$$
R_{ij}=\begin{cases}R_{ij}^*&fit_i(R_{ij}^*)\geqslant fit_i(R_{ij})\\R_{ij}&other \end{cases}\tag{4}
$$




（3）所有雇佣蜂进行$$SP$$次操作后，飞回信息交流区，观察蜂根据信息，根据一定概率通过「轮盘赌」选择食物源(即为跟随过程)，由下式计算概率：


$$
p_i={fit_i\over \sum_{i=1}^{SP}fit_i}\tag{5}
$$


选择食物源后，观察蜂在其邻近区域进行探索，由(2)式在其周围产生一个新的食物源，并按照同样的(3)式和(4)式进行贪婪选择较优蜜源。

（4）观察蜂探索(迭代)过程中，若蜜源 $$R_i$$ 的迭代次数 $$trail$$ 达到阈值 $$limit$$ 而没有经过一次更新(局部最优)，则该蜜源 $$R_i$$ 将被舍弃，与之对应的雇佣蜂转变为侦查蜂，在搜索空间中由(1)式随机产生新蜜源(解)：


$$
R_{ij}^{t+1}=\begin{cases}(1)formula&trail\geqslant limit\\(2)formula&traill\lt limit \end{cases}\tag{6}
$$


（5）到达终止条件，得到全局最优解。

## 3.模型建立及代码实现

C++实例：

```C++
#include<iostream>
#include<time.h>
#include<stdlib.h>
#include<cmath>
#include<fstream>
#include<iomanip>
using namespace std;

const int NP=40;//种群的规模，采蜜蜂+观察蜂
const int FoodNumber=NP/2;//食物的数量，为采蜜蜂的数量
const int limit=20;//限度，超过这个限度没有更新采蜜蜂变成侦查蜂
const int maxCycle=10000;//停止条件

/*****函数的特定参数*****/
const int D=2;//函数的参数个数
const double lb=-100;//函数的下界
const double ub=100;//函数的上界

double result[maxCycle]={0};

/*****种群的定义****/
struct BeeGroup
{
    double code[D];//函数的维数
    double trueFit;//记录真实的最小值
    double fitness;
    double rfitness;//相对适应值比例
    int trail;//表示实验的次数，用于与limit作比较
}Bee[FoodNumber];

BeeGroup NectarSource[FoodNumber];//蜜源，注意：一切的修改都是针对蜜源而言的
BeeGroup EmployedBee[FoodNumber];//采蜜蜂
BeeGroup OnLooker[FoodNumber];//观察蜂
BeeGroup BestSource;//记录最好蜜源

/*****函数的声明*****/
double random(double, double);//产生区间上的随机数
void initilize();//初始化参数
double calculationTruefit(BeeGroup);//计算真实的函数值
double calculationFitness(double);//计算适应值
void CalculateProbabilities();//计算轮盘赌的概率
void evalueSource();//评价蜜源
void sendEmployedBees();
void sendOnlookerBees();
void sendScoutBees();
void MemorizeBestSource();


/*******主函数*******/
int main()
{
    ofstream output;  //输出定义
    output.open("dataABC.txt");

    srand((unsigned)time(NULL));  //根据时间产生随机种子
    initilize();//初始化
    MemorizeBestSource();//保存最好的蜜源

    //主要的循环
    int gen=0;
    while(gen<maxCycle)
    {
        sendEmployedBees();

        CalculateProbabilities();

        sendOnlookerBees();

        MemorizeBestSource();

        sendScoutBees();

        MemorizeBestSource();

        output<<setprecision(30)<<BestSource.trueFit<<endl;  //输出30个有效数字

        gen++;
    }

    output.close();
    cout<<"运行结束!!"<<endl;
    return 0;
}

/*****函数的实现****/
double random(double start, double end)//随机产生区间内的随机数
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

void initilize()//初始化参数
{
    int i,j;
    for (i=0;i<FoodNumber;i++)
    {
        for (j=0;j<D;j++)
        {
            NectarSource[i].code[j]=random(lb,ub);
            EmployedBee[i].code[j]=NectarSource[i].code[j];
            OnLooker[i].code[j]=NectarSource[i].code[j];
            BestSource.code[j]=NectarSource[0].code[j];
        }
        /****蜜源的初始化*****/
        NectarSource[i].trueFit=calculationTruefit(NectarSource[i]);
        NectarSource[i].fitness=calculationFitness(NectarSource[i].trueFit);
        NectarSource[i].rfitness=0;
        NectarSource[i].trail=0;
        /****采蜜蜂的初始化*****/
        EmployedBee[i].trueFit=NectarSource[i].trueFit;
        EmployedBee[i].fitness=NectarSource[i].fitness;
        EmployedBee[i].rfitness=NectarSource[i].rfitness;
        EmployedBee[i].trail=NectarSource[i].trail;
        /****观察蜂的初始化****/
        OnLooker[i].trueFit=NectarSource[i].trueFit;
        OnLooker[i].fitness=NectarSource[i].fitness;
        OnLooker[i].rfitness=NectarSource[i].rfitness;
        OnLooker[i].trail=NectarSource[i].trail;
    }
    /*****最优蜜源的初始化*****/
    BestSource.trueFit=NectarSource[0].trueFit;
    BestSource.fitness=NectarSource[0].fitness;
    BestSource.rfitness=NectarSource[0].rfitness;
    BestSource.trail=NectarSource[0].trail;
}

double calculationTruefit(BeeGroup bee)//计算真实的函数值
{
    double truefit=0;
    /******测试函数1******/
    truefit=0.5+(sin(sqrt(bee.code[0]*bee.code[0]+bee.code[1]*bee.code[1]))*sin(sqrt(bee.code[0]*bee.code[0]+bee.code[1]*bee.code[1]))-0.5)
        /((1+0.001*(bee.code[0]*bee.code[0]+bee.code[1]*bee.code[1]))*(1+0.001*(bee.code[0]*bee.code[0]+bee.code[1]*bee.code[1])));

    return truefit;
}

double calculationFitness(double truefit)//计算适应值
{
    double fitnessResult=0;
    if (truefit>=0)
    {
        fitnessResult=1/(truefit+1);
    }else
    {
        fitnessResult=1+abs(truefit);
    }
    return fitnessResult;
}

void sendEmployedBees()//修改采蜜蜂的函数
{
    int i,j,k;
    int param2change;//需要改变的维数
    double Rij;//[-1,1]之间的随机数
    for (i=0;i<FoodNumber;i++)
    {

        param2change=(int)random(0,D);//随机选取需要改变的维数

        /******选取不等于i的k********/
        while (1)
        {
            k=(int)random(0,FoodNumber);
            if (k!=i)
            {
                break;
            }
        }

        for (j=0;j<D;j++)
        {
            EmployedBee[i].code[j]=NectarSource[i].code[j];  //在之前初始化对EmployedBee进行初始化了，程序之后有对蜜源改变在此对EmployedBee进行更新
        }

        /*******采蜜蜂去更新信息*******/
        Rij=random(-1,1);
        EmployedBee[i].code[param2change]=NectarSource[i].code[param2change]+Rij*(NectarSource[i].code[param2change]-NectarSource[k].code[param2change]);  //根据公式(2-3)
        /*******判断是否越界********/
        if (EmployedBee[i].code[param2change]>ub)
        {
            EmployedBee[i].code[param2change]=ub;
        }
        if (EmployedBee[i].code[param2change]<lb)
        {
            EmployedBee[i].code[param2change]=lb;
        }
        EmployedBee[i].trueFit=calculationTruefit(EmployedBee[i]);
        EmployedBee[i].fitness=calculationFitness(EmployedBee[i].trueFit);

        /******贪婪选择策略*******/
        if (EmployedBee[i].trueFit<NectarSource[i].trueFit)
        {
            for (j=0;j<D;j++)
            {
                NectarSource[i].code[j]=EmployedBee[i].code[j];
            }
            NectarSource[i].trail=0;
            NectarSource[i].trueFit=EmployedBee[i].trueFit;
            NectarSource[i].fitness=EmployedBee[i].fitness;
        }else
        {
            NectarSource[i].trail++;
        }
    }
}

void CalculateProbabilities()//计算轮盘赌的选择概率  (计算的适应度比例)与后面 sendOnlookerBees中的选择R_choosed进行比较
{
    int i;
    double maxfit;
    maxfit=NectarSource[0].fitness;
    for (i=1;i<FoodNumber;i++)
    {
        if (NectarSource[i].fitness>maxfit)
            maxfit=NectarSource[i].fitness;
    }

    for (i=0;i<FoodNumber;i++)
    {
        NectarSource[i].rfitness=(0.9*(NectarSource[i].fitness/maxfit))+0.1;
    }
}

void sendOnlookerBees()//采蜜蜂与观察蜂交流信息，观察蜂更改信息
{
    int i,j,t,k;
    double R_choosed;//被选中的概率
    int param2change;//需要被改变的维数
    double Rij;//[-1,1]之间的随机数
    i=0;
    t=0;  //是否超出食物源个数
    while(t<FoodNumber)
    {

        R_choosed=random(0,1);
        if(R_choosed<NectarSource[i].rfitness)//根据被选择的概率选择  （算法搜索过程三的实现）
        {
            t++;
            param2change=(int)random(0,D);

            /******选取不等于i的k********/
            while (1)
            {
                k=(int)random(0,FoodNumber);
                if (k!=i)
                {
                    break;
                }
            }

            for(j=0;j<D;j++)
            {
                OnLooker[i].code[j]=NectarSource[i].code[j];
            }

            /****更新******/
            Rij=random(-1,1);
            OnLooker[i].code[param2change]=NectarSource[i].code[param2change]+Rij*(NectarSource[i].code[param2change]-NectarSource[k].code[param2change]);

            /*******判断是否越界*******/
            if (OnLooker[i].code[param2change]<lb)
            {
                OnLooker[i].code[param2change]=lb;
            }
            if (OnLooker[i].code[param2change]>ub)
            {
                OnLooker[i].code[param2change]=ub;
            }
            OnLooker[i].trueFit=calculationTruefit(OnLooker[i]);
            OnLooker[i].fitness=calculationFitness(OnLooker[i].trueFit);

            /****贪婪选择策略******/
            if (OnLooker[i].trueFit<NectarSource[i].trueFit)
            {
                for (j=0;j<D;j++)
                {
                    NectarSource[i].code[j]=OnLooker[i].code[j];
                }
                NectarSource[i].trail=0;
                NectarSource[i].trueFit=OnLooker[i].trueFit;
                NectarSource[i].fitness=OnLooker[i].fitness;
            }else
            {
                NectarSource[i].trail++;
            }
        }
        i++;
        if (i==FoodNumber)
        {
            i=0;
        }
    }
}


/*******只有一只侦查蜂（主程序进行一次循环该函数就判断一次侦查蜂是否出现）（如果有两个及以上超过limit限制，该函数能将他们都找到吗？在循环结束之前）**********/
void sendScoutBees()//判断是否有侦查蜂的出现，有则重新生成蜜源
{
    int maxtrialindex,i,j;
    double R;//[0,1]之间的随机数
    maxtrialindex=0;
    for (i=1;i<FoodNumber;i++)
    {
        if (NectarSource[i].trail>NectarSource[maxtrialindex].trail)
        {
            maxtrialindex=i;
        }
    }
    if(NectarSource[maxtrialindex].trail>=limit)
    {
        /*******重新初始化*********/
        for (j=0;j<D;j++)
        {
            R=random(0,1);
            NectarSource[maxtrialindex].code[j]=lb+R*(ub-lb);  //此处蜜源进行更新（这就是为什么在初始化引领蜂和侦查蜂之后，还要在相应的函数地方再初始化一次）
        }
        NectarSource[maxtrialindex].trail=0;
        NectarSource[maxtrialindex].trueFit=calculationTruefit(NectarSource[maxtrialindex]);
        NectarSource[maxtrialindex].fitness=calculationFitness(NectarSource[maxtrialindex].trueFit);
    }
}

void MemorizeBestSource()//保存最优的蜜源
{
    int i,j;
    for (i=1;i<FoodNumber;i++)
    {
        if (NectarSource[i].trueFit<BestSource.trueFit)
        {
            for (j=0;j<D;j++)
            {
                BestSource.code[j]=NectarSource[i].code[j];
            }
            BestSource.trueFit=NectarSource[i].trueFit;
        }
    }
}
```

MATLAB实例：

```MATLAB
% 随机生成（-10 10）之间的5个数，使5个数的平方和最小
clc;
clear;
close all;
% 问题定义
CostFunction=@(x) Sphere(x);%定义关于x的函数
nVar=5;
VarSize=[1 5];
VarMin=-20;
VarMax=20;
% ABC参数定义
MaxIt=2000; % 种群迭代次数
nPop=100;  % 雇佣蜂数量
nOnlooker=nPop;   %观察蜂数量
L=round(0.6*nVar*nPop);  %蜜源试验限制，侦查蜂判断
a=1;  % 加速系数的最大值
%% 初始化时期
empty_bee.Position=[];
empty_bee.Cost=[];  % 蜜源
pop=repmat(empty_bee,nPop,1);
BestSol.Cost=inf;%正无穷
for i=1:nPop
    pop(i).Position=unifrnd(VarMin,VarMax,VarSize);%产生随机数
    pop(i).Cost=CostFunction(pop(i).Position);%计算解的价值
    if pop(i).Cost<BestSol.Cost%进行选择
        BestSol=pop(i)
    end
end
C=zeros(nPop,1); % 用来判断蜜源的试验限制
BestCost=zeros(MaxIt,1);  %用来记录迄今为止最好的蜜源
 
for it=1:MaxIt%一直到迭代次数达到MaxIt结束
    % 雇佣蜂时期
    for i=1:nPop
        K=[1:i-1 i+1:nPop];%不包括i
        k=K(randi([1 numel(K)]));
        phi=a*unifrnd(-1,1,VarSize); % 加速系数
        newbee.Position=pop(i).Position+phi.*(pop(i).Position-pop(k).Position);%寻找新解(蜜源)
        newbee.Cost=CostFunction(newbee.Position);%计算新解(蜜源)的价值
        if newbee.Cost<=pop(i).Cost%进行选择
            pop(i)=newbee;
        else
            C(i)=C(i)+1;%用来判断蜜源的试验限制，超过一定次数未更新即舍弃
        end
    end
    F=zeros(nPop,1);%nPop个跟随蜂的适应系数
    MeanCost=mean([pop.Cost]);%计算pop中所有cost的均值
    for i=1:nPop
        F(i)=exp(-pop(i).Cost/MeanCost); % 适应函数
    end
    P=F/sum(F);%概率向量
    % 观察蜂时期
    for m=1:nOnlooker
        i=RouletteWheelSelection(P);  % 采用轮盘赌随机选择一个蜜源
        K=[1:i-1 i+1:nPop];
        k=K(randi([1 numel(K)]));
        phi=a*unifrnd(-1,1,VarSize);
        newbee.Position=pop(i).Position+phi.*(pop(i).Position-pop(k).Position);
        newbee.Cost=CostFunction(newbee.Position);
        if newbee.Cost<=pop(i).Cost
            pop(i)=newbee;
        else
            C(i)=C(i)+1;
        end
    end
    % 侦查蜂
    for i=1:nPop
        if C(i)>=L   % 判断蜜源是否到达试验限制，到达限制后重开
            pop(i).Position=unifrnd(VarMin,VarMax,VarSize);
            pop(i).Cost=CostFunction(pop(i).Position);
            C(i)=0;
        end
    end
    
    for i=1:nPop
        if pop(i).Cost<=BestSol.Cost
            BestSol=pop(i);  %寻找最佳蜜源
        end
    end
    BestCost(it)=BestSol.Cost;  % 存储最佳蜜源
    disp(['Iteration ' num2str(it) ': Best Cost= ' num2str(BestCost(it))]);
end
 
figure;
semilogy(BestCost,'lineWidth',2)
xlabel('Iteration');
ylabel('Best Cost');
grid on;
 
% 代价函数
function z=Sphere(x)
    z=sum(x.^2);
end
 
% 轮盘赌算法
function i=RouletteWheelSelection(P)
    r=rand;  
    C=cumsum(P);%计算各行的累加值
    i=find(r<=C,1,'first');
end
```

## 4.算法改进

[^1]: 引自zhuanlan.zhihu.com/p/253220840
[^2]: 莫建麟,王玉晶,基于智能搜索和特殊划分的人工蜂群算法,重庆邮电大学学报,2020(12):1081-1082.
[^3]: 王淑芬,原杨飞,改进的人工蜂群算法求解全局优化问题,宝鸡文理学院学报,2021(6),24-25.
