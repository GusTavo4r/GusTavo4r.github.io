# 启发式算法-遗传算法（GA）


## 1.简述

遗传算法（Genetic Algorithm，GA）根据大自然中生物体进化规律而提出，是模拟达尔文生物进化论的自然选择和遗传学机理的生物进化过程的计算模型，是一种通过模拟自然进化过程搜索最优解的方法。该算法通过数学的方式,利用计算机仿真运算,将问题的求解过程转换成类似生物进化中的染色体基因的交叉、变异等过程。在求解较为复杂的组合优化问题时,相对一些常规的优化算法,通常能够较快地获得较好的优化结果[^1]。 

所有生物的进化都具有一些共同的特点：

- 进化的发生地点是信息载体，即染色体（Chromosome），而不是被编码的生物个体（Individual）上。
- 染色体能被复制，成功适应环境的被编码个体的染色体有更多的机会进行复制，形成有优势的后代。
- 染色体能产生突变，使子代染色体不同于母代。

其主要特点是直接对结构对象进行操作，不存在求导和函数连续性的限定；具有内在的隐并行性和更好的全局寻优能力；采用概率化的寻优方法，不需要确定的规则就能自动获取和指导优化的搜索空间，自适应地调整搜索方向。

## 2.优化过程

### 2.1原理简述

与自然界相似，遗传算法对求解问题的本身一无所知，它所需要的仅是对算法所产生的每个染色体进行评价，并基于适应值来选择染色体，使适应性好的染色体有更多的繁殖机会。在遗传算法中，通过随机方式产生若干个所求解问题的数字「编码」，即染色体，形成初始群体；通过「适应度」函数给每个个体一个数值评价，淘汰低适应度的个体，选择高适应度的个体参加遗传操作，经过遗传操作后的个体集合形成下一代新的种群。对这个新种群进行下一轮进化。 

遗传算法采用种群的方式组织搜索，因而可同时搜索解空间内的多个区域，并相互交流信息。遗传算法是一种可以克服搜索空间过大、寻求最优解的近似解的启发式算法。

遗传算法的基本步骤：

1. 设定初始种群(初始解集)；
2. 计算种群中每个个体(染色体)的适应度，根据适应度对个体进行「选择」；
3. 选择后个体的染色体发生「交叉」和「变异」现象，产生新的下一代种群；
4. 进入下一代，重复前三步；
5. 直到达到终止条件，结束，输出种群中适应度最优的个体作为问题的满意解。

### 2.2重要步骤

#### 2.2.1基因及染色体

待解问题的一个可行解为一条染色体(Chromosome)，GA求解空间中表示为一条位串(Bit string)；解中包含的多个元素为其基因(Gene)，表示为单个字符或数字。

#### 2.2.2编码及解码

编码是指把一个问题的可行解从其解空间转换到遗传算法的搜索空间的转换方法。解码（译码）是指遗传算法解空间向问题空间的转换。 编码通常需满足几个规范：(1)完备性(completeness);(2)健全性(soundness);(3)非冗余性(nonredundancy)[^2]。常见编码方法：二进制编码、格雷码编码、 浮点数编码等等。

以二进制编码为例[[^3]：

设某一参数值的取值范围为 $$[U_1,U_2]$$ ，将其映射为长度为 $$k$$ 的二进制编码串，总共产生 $$2^k$$ 个二进制串，其对应关系为：


$$
\begin{aligned}
&00000\cdots 000 (0)\rightarrow U_1\\
&00001\cdots 000 (1)\rightarrow U_1+\sigma\\
&00010\cdots 000 (2)\rightarrow U_1+2\sigma\\
&\vdots\\
&11111\cdots 111 (2^k-1)\rightarrow U_2&其中\sigma={U_2-U_1\over2}
\end{aligned}
$$


解码过程为：


$$
X=U_1+(\sum_{i=1}^kb_i\cdot2^{i-1})\cdot{U_2-U_1\over2^k-1}\tag{1}
$$


其中 $$\sum_{i=1}^kb_i\cdot2^{i-1}$$ 为二进制串对应的十进制值。

#### 2.2.3遗传算子

**（1）选择(selection)**

选择操作从旧群体中以一定概率选择优良个体(即染色体)组成新的种群，用来确定重组或交叉个体，以及被选个体将产生多少个子代个体，以得到下一代种群。个体被选中的概率跟适应度值有关，个体适应度值越高，被选中的概率越大。 

常用的选择算子：

1. 轮盘赌选择Roulette Wheel Selection：最常用的选择算子。这是一种回放式随机采样方法。每个个体进入下一代的概率等于它的适应度值与整个种群中个体适应度值和的比例。选择误差较大。
2. 随机竞争选择Stochastic Tournament ：每次按轮盘赌选择一对个体，然后让这两个个体进行竞争，适应度高的被选中，如此反复，直到选满为止。
3. 最佳保留选择：首先按轮盘赌选择方法执行遗传算法的选择操作，然后将当前群体中适应度最高的个体结构完整地复制到下一代群体中。
4. 无回放随机选择（也叫期望值选择Excepted Value Selection）：根据每个个体在下一代群体中的生存期望来进行随机选择运算。方法如下：（1）计算群体中每个个体在下一代群体中的生存期望数目N；（2）若某一个体被选中参与交叉运算，则它在下一代中的生存期望数目减去0.5，若某一个体未被选中参与交叉运算，则它在下一代中的生存期望数目减去1.0；（3）随着选择过程的进行，若某一个体的生存期望数目小于0时，则该个体就不再有机会被选中。

**（2）交叉(crossover)**

交叉操作是指从种群中随机选择两个个体，通过两个染色体的交换组合，把父串的优秀特征遗传给子串，从而产生新的优秀个体。对「单点交叉」算子来说，其过程为：对种群个体进行随机配对；逐一对配对的染色体，随机设置一个交叉点；按设定的交叉概率进行交换。其余还有双点交叉或多点交叉、均匀交叉、算术交叉等等。

**（3）变异(mutation)**

为了防止遗传算法在优化过程中陷入局部最优解，在搜索过程中，需要对个体进行变异，在实际应用中，主要采用「单点变异」，也叫位变异，即只需要对基因序列中某一个位进行变异，以二进制编码为例，即0变为1，而1变为0。 

#### 2.2.4适应度函数

适应度函数也称评价函数，是根据目标函数确定的用于区分群体中个体好坏的标准。适应度函数总是非负的，而目标函数可能有正有负，故需要在目标函数与适应度函数之间进行变换。 

评价个体适应度的一般过程为：

1. 对个体 $$x_i$$ 编码串(基因型)进行解码处理后，可得到个体的表现型；
2. 由个体的表现型及对应关系 $$F$$ 可计算出对应个体的目标函数值 $$F(x_i)$$；
3. 根据最优化问题的类型，由目标函数值按一定的转换规则 $$G$$ 求出个体的适应度 $$G(F(x_i))=Fit(x_i)$$。

由目标函数到适应度函数的转换

- $$Fit(x_i)=F(x_i)$$，目标函数为最大化问题
- $$Fit(x_i)=-F(x_i)$$，目标函数为最小化问题

#### 2.2.5终止条件

程序的终止条件有两种：完成了预先设定的最大进化迭代数时；种群中的最优个体在连续若干代没有改进（最优适应度没有改进）或平均适应度在连续若干代基本没有改进时。

> 需要预先设定的参数：
>
> $$M$$ ：种群大小；$$T$$ ：遗传算法的最大迭代次数；$$P_c$$ ：交叉概率；$$P_m$$ ：变异概率。

## 2.具体应用

### 2.1负载均衡调度问题

问题描述：假设有 $$N$$ 个任务，需要负载均衡器分配给 $$M$$ 个服务器节点去处理。每个任务的任务长度、每台服务器节点(下面简称“节点”)的处理速度已知，请给出一种任务分配方式，使得所有任务的总处理时间最短。 

### 2.2混合流水车间调度问题(Hybrid Flow-shop Scheduling Problem, HFSP )[^4]

**假设和约束**

1. 一个工件在一道工序上被任意一个机器加工。
2. 一个机器在某一时刻只能空闲或加工一个工件。
3. 工件必须按照加工工序顺序进行加工。
4. 同一道工序中机器都相同。
5. 工件加工过程不允许中断。
6. 如果多个工件同时需要被加工，则按优先级顺序进行加工。
7. 同一台机器可同时完成任意工序，但同一工序同一时刻只能加工一个工件。

**需要解决的问题**：确定零件的加工优先级，以使整体加工时间最短。

以2台机器，6个工件，3道工序为例：

**(1)C++代码实现：**

按照以下格式输入各工件每道工序的加工时间（input.txt）：


$$
\begin{array}{c c c c}\hline&工序1&工序2&工序3\\工件1&8&4&6\\工件2&2&9&7\\工件3&4&2&8\\工件4&3&1&6\\工件5&5&2&10\\工件6&9&5&3\\\hline\\ \end{array}
$$

```C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <time.h>
#include <algorithm>
using namespace std;
ofstream outfile;
#define machinenumber 6  //机器的总数（等于每道工序的并行机个数×工序数）
#define parallel 2       //每道工序的并行机个数
#define ordernumber 3     //工序数
#define workpiecesnumber 6  //工件总数
#define populationnumber 200  //每一代种群的个体数

double crossoverrate=0.6;            //交叉概率
double mutationrate=0.05;             //变异概率
int G=100;                        //循环代数100
int usetime[workpiecesnumber][ordernumber];  //第几个工件第几道工序的加工用时；
int machinetime[ordernumber][parallel]= {0}; //第几道工序的第几台并行机器的统计时间；
int starttime[workpiecesnumber][ordernumber][parallel];//第几个工件第几道工序在第几台并行机上开始加工的时间；
int finishtime [workpiecesnumber][ordernumber][parallel];//第几个工件第几道工序在第几台并行机上完成加工的时间；
int ttime[populationnumber];      //个体的makespan；                                                    ？？？？？？？？？？？？？？？？？？？？？？？
int a [populationnumber][workpiecesnumber];//第几代的染色体顺序，即工件加工顺序；
int times[100];  //用来存储已知用时的数组；
int makespan;    //总的流程加工时间；
int flg7;   //暂时存储流程加工时间；
double fits[populationnumber] ;//存储每一代种群每一个个体的适应度，便于进行选择操作；
                                                                                       //？？？？？？？？？？？？？？？？
int initialization()   //初始化种群；
{
    for(int i=0; i<populationnumber; i++)     //首先生成一个工件个数的全排列的个体；
        for(int j=0; j<workpiecesnumber; j++)
        {
            a[i][j]=j+1;
        }
        
    for(int i=0; i<populationnumber; i++)     //将全排列的个体中随机选取两个基因位交换，重复工件个数次，以形成随机初始种群；
        for(int j=0; j<workpiecesnumber; j++)
        {
            int flg1=rand()%workpiecesnumber;
            int flg2=rand()%workpiecesnumber;
            int flg3=a[i][flg1];
            a[i][flg1]=a[i][flg2];
            a[i][flg2]=flg3;
        }
        
    for(int i=0; i<populationnumber; i++)
    {
        for(int j=0; j<workpiecesnumber; j++)
        {
            cout<<a[i][j]<<" ";
        }
        cout<<endl;
    }
    return 0;
}

int fitness(int c)   //计算适应度函数，c代表某个体；
{
    int totaltime;      //总的加工流程时间（makespan）；
    int temp1[workpiecesnumber]= {0};
    int temp2[workpiecesnumber]= {0};
    int temp3[workpiecesnumber]= {0};

    for(int j=0; j<workpiecesnumber; j++)   //temp1暂时存储个体c的基因序列，以便进行不同流程之间的加工时记录工件加工先后顺序；
    {
        temp1[j]=a[c][j];
    }

    for(int i=0; i<ordernumber; i++)
    {
        for(int j=0; j<workpiecesnumber; j++)  //该循环的目的是通过比较所有机器的当前工作时间，找出最先空闲的机器，便于新的工件生产；
        {
            int m=machinetime[i][0];        //先记录第i道工序的第一台并行机器的当前工作时间；
            int n=0;
            for (int p=0; p<parallel; p++) //与其他并行机器进行比较，找出时间最小的机器；
            {
                if (m>machinetime[i][p])
                {
                    m=machinetime[i][p];
                    n=p;
                }
            }
            int q=temp1[j];                 //按顺序提取temp1中的工件号，对工件进行加工；
            starttime[q-1][i][n]=max(machinetime[i][n],temp3[j]);  //开始加工时间取该机器的当前时间和该工件上一道工序完工时间的最大值；
            machinetime[i][n]=starttime[q-1][i][n]+usetime[q-1][i] ; //机器的累计加工时间等于机器开始加工的时刻，加上该工件加工所用的时间；
            finishtime[q-1][i][n]=machinetime[i][n];                 //工件的完工时间就是该机器当前的累计加工时间；
            temp2[j]=finishtime[q-1][i][n];       //将每个工件的完工时间赋予temp2，根据完工时间的快慢，便于决定下一道工序的工件加工顺序；
        }

        int flg2[workpiecesnumber]= {0};           //生成暂时数组，便于将temp1和temp2中的工件重新排列；
        for(int s=0; s<workpiecesnumber; s++)
        {
            flg2[s]=temp1[s];
        }

        for (int e=0; e<workpiecesnumber-1; e++)
        {
            for(int ee=0; ee<workpiecesnumber-1-e; ee++) // 由于temp2存储工件上一道工序的完工时间，在进行下一道工序生产时，按照先完工先生产的
            {                                            //原则，因此，该循环的目的在于将temp2中按照加工时间从小到大排列，同时temp1相应进行变换
                if (temp2[ee]>temp2[ee+1])               //来记录temp2中的工件号；
                {
                    int flg5=temp2[ee];
                    int flg6=flg2[ee];
                    temp2[ee]=temp2[ee+1];
                    flg2[ee]=flg2[ee+1];
                    temp2[ee+1]=flg5;
                    flg2[ee+1]=flg6;
                }
            }
        }
        for(int e=0; e<workpiecesnumber; e++)    //更新temp1，temp2的数据，开始下一道工序；
        {
            temp1[e]=flg2[e];
            temp3[e]=temp2[e];
        }
    }
    totaltime=0;
    for (int i=0; i<parallel; i++) //比较最后一道工序机器的累计加工时间，最大时间就是该流程的加工时间；
        if (totaltime<machinetime[ordernumber-1][i])
        {
            totaltime=machinetime[ordernumber-1][i];
        }
    for(int i=0; i<workpiecesnumber; i++)  //将数组归零，便于下一个个体的加工时间统计；
        for(int j=0; j<ordernumber; j++)
            for(int t=0; t<parallel; t++)
            {
                starttime[i][j][t]=0;
                finishtime[i][j][t]=0;
                machinetime[j][t]=0;
            }
    makespan=totaltime;
    fits[c]=1.000/makespan;          //将makespan取倒数作为适应度函数；
}



int gant(int c)                   //该函数是为了将最后的结果便于清晰明朗的展示并做成甘特图，对问题的结果以及问题的解决并没有影响；
{

    int totaltime;
    char machine[ordernumber*parallel][100]= {"0"};

    int temp1[workpiecesnumber]= {0}; //jiagongshunxu
    int temp2[workpiecesnumber]= {0}; //shangyibuzhou de wan cheng shijian
    int temp3[workpiecesnumber]= {0};

    //////////////////////////////////////////
    for(int j=0; j<workpiecesnumber; j++)
    {
        temp1[j]=a[c][j];
    }
    for(int i=0; i<ordernumber; i++)

    {


        for(int j=0; j<workpiecesnumber; j++)
        {

            int m=machinetime[i][0];
            int n=0;

            for (int p=0; p<parallel; p++) //找出时间最小的机器；
            {
                if (m>machinetime[i][p])
                {
                    m=machinetime[i][p];
                    n=p;

                }
            }
            int q=temp1[j];

            starttime[q-1][i][n]=max(machinetime[i][n],temp3[j]);
            machinetime[i][n]=starttime[q-1][i][n]+usetime[q-1][i] ;
            finishtime[q-1][i][n]=machinetime[i][n];
            temp2[j]=finishtime[q-1][i][n];
            //cout<<"start:"<<starttime[q-1][i][n]<<"   use:"<<usetime[q-1][i]<<"  machine:"<<machinetime[i][n]<<"   finish:"<<finishtime[q-1][i][n]<<endl;
            for(int h=starttime[q-1][i][n]; h<finishtime[q-1][i][n]; h++)
            {
                if (q==1)
                    machine[i*2+n][h]='1';
                else if (q==2)
                    machine[i*2+n][h]='2';
                else if (q==3)
                    machine[i*2+n][h]='3';
                else if (q==4)
                    machine[i*2+n][h]='4';
                else if (q==5)
                    machine[i*2+n][h]='5';
                else
                    machine[i*2+n][h]='6';
            }

        }

        int flg2[workpiecesnumber]= {0};
        for(int s=0; s<workpiecesnumber; s++)
        {
            flg2[s]=temp1[s];
        }
        for (int e=0; e<workpiecesnumber-1; e++)
        {
            for(int ee=0; ee<workpiecesnumber-1-e; ee++)
            {
                if (temp2[ee]>temp2[ee+1])
                {
                    int flg5=temp2[ee];
                    int flg6=flg2[ee];
                    temp2[ee]=temp2[ee+1];
                    flg2[ee]=flg2[ee+1];
                    temp2[ee+1]=flg5;
                    flg2[ee+1]=flg6;
                    //swap(temp2[ee],temp2[ee+1]);
                    //swap(flg2[ee],flg2[ee+1]);
                }
            }
        }

        for(int e=0; e<workpiecesnumber; e++)
        {
            temp1[e]=flg2[e];
            temp3[e]=temp2[e];
            //cout<<"temp3=="<<temp3[e]<<endl;
        }
    }

    totaltime=0;
    for (int i=0; i<parallel; i++)
        if (totaltime<machinetime[ordernumber-1][i])
        {
            totaltime=machinetime[ordernumber-1][i];

        }
      cout<<"total="<<totaltime<<endl;
      outfile<<totaltime<<endl;///////////////////////////////////////////////////////////////////////////////
    flg7=totaltime;
    for(int u=0; u<ordernumber*parallel; u++)
    {
        for(int uu=0; uu<100; uu++)
            {outfile<<machine[u][uu];
            cout<<machine[u][uu];
            }
            outfile<<endl;
            cout<<endl;
    }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int select()
{
   double roulette[populationnumber+1]={0.00};	//记录轮盘赌的每一个概率区间；
   double pro_single[populationnumber];			//记录每个个体出现的概率，即个体的适应度除以总体适应度之和；
   double totalfitness=0.00;                	//种群所有个体的适应度之和；
   int a1[populationnumber][workpiecesnumber];	//存储a中所有个体的染色体；

   for(int i=0;i<populationnumber;i++)     		//计算所有个体适应度的总和；
   {
       totalfitness=totalfitness+fits[i];
   }

   for(int i=0;i<populationnumber;i++)
   {
       pro_single[i]=fits[i]/totalfitness;   	//计算每个个体适应度与总体适应度之比；
       roulette[i+1]=roulette[i]+pro_single[i]; //将每个个体的概率累加，构造轮盘赌；
   }

   for(int i=0;i<populationnumber;i++)
   {
       for(int j=0;j<workpiecesnumber;j++)
       {
           a1[i][j]=a[i][j];               //a1暂时存储a的值；
       }
   }

   for(int i=0;i<populationnumber;i++)
   {
       int a2;   //当识别出所属区间之后，a2记录区间的序号；
       double p=rand()%(1000)/(double)(1000);
       for(int j=0;j<populationnumber;j++)
       {
           if(p>=roulette[j]&&p<roulette[j+1])
              a2=j;
       }
       for(int m=0;m<workpiecesnumber;m++)
       {
           a[i][m]=a1[a2][m];
       }
   }


}

int crossover()
/*种群中的个体随机进行两两配对，配对成功的两个个体作为父代1和父代2进行交叉操作。
随机生成两个不同的基因点位，子代1继承父代2基因位之间的基因片段，其余基因按顺序集成父代1中未重复的基因；
子代2继承父代1基因位之间的基因片段，其余基因按顺序集成父代2中未重复的基因。*/
{
    for(int i=0;i<populationnumber/2;i++) //将所有个体平均分成两部分，一部分为交叉的父代1，一部分为进行交叉的父代2；
    {
        int n1=1+rand()%workpiecesnumber/2;    //该方法生成两个不同的基因位；
        int n2=n1+rand()%(workpiecesnumber-n1-1)+1;
        int n3=rand()%10;
        if(n3==2)   //n3=2的概率为0.1；若满足0.1的概率，那么就进行交叉操作；
        {
            int temp1[workpiecesnumber]={0};int temp2[workpiecesnumber]={0};
            for(int j=0;j<workpiecesnumber;j++)
            {
                int flg1=0;int flg2=0;
                for(int p=n1;p<n2;p++)          //将交叉点位之间的基因片段进行交叉，temp1和temp2记录没有发生重复的基因；
                    {if(a[2*i+1][p]==a[2*i][j])
                      flg1=1;
                    }
                    if(flg1==0){temp1[j]=a[2*i][j];}

                for(int p=n1;p<n2;p++)
                    {if(a[2*i][p]==a[2*i+1][j])
                    flg2=1;
                    }
                    if(flg2==0){temp2[j]=a[2*i+1][j];}

            }


            for(int j=n1;j<n2;j++)             //子代1继承父代2交叉点位之间的基因；子代2继承父代1交叉点位之间的基因；
            {
                int n4=0;
                n4=a[2*i][j];
                a[2*i][j]=a[2*i+1][j];
                a[2*i+1][j]=n4;
            }
            for(int p=0;p<n1;p++)               //子代1第一交叉点之前的基因片段，按顺序依次继承父代1中未与子代1重复的基因；
            {
                for( int q=0;q<workpiecesnumber;q++)
                    {if(temp1[q]!=0)
                      {a[2*i][p]=temp1[q];temp1[q]=0;
                       break;}
                    }
            }
            for(int p=0;p<n1;p++)               //子代2第一交叉点之前的基因片段，按顺序依次继承父代2中未与子代2重复的基因；
            {
                for( int m=0;m<workpiecesnumber;m++)
                    {if(temp2[m]!=0)
                     {a[2*i+1][p]=temp2[m];temp2[m]=0;
                      break;}
                    }
            }
            for(int p=n2;p<workpiecesnumber;p++)             //子代1第2交叉点之后的基因片段，按顺序依次继承父代1中未与子代1重复的基因；
            {
                for( int q=0;q<workpiecesnumber;q++)
                    {if(temp1[q]!=0)
                      {a[2*i][p]=temp1[q];temp1[q]=0;
                       break;}

                    }
            }
            for(int p=n2;p<workpiecesnumber;p++)               //子代2第2交叉点之后的基因片段，按顺序依次继承父代2中未与子代2重复的基因；
                {
                    for( int m=0;m<workpiecesnumber;m++)
                    {if(temp2[m]!=0)
                     {a[2*i+1][p]=temp2[m];temp2[m]=0;
                     break;}
                    }
                }

        }
    }


}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int mutation()  //变异操作为两点变异，随机生成两个基因位，并交换两个基因的位置；
{
    int n3=rand()%20;
        if(n3==2)
        {
           for(int i=0; i<populationnumber; i++)
          {
           int b1=rand()%workpiecesnumber;
           int b2=rand()%workpiecesnumber;
           int b3=a[i][b1];
           a[i][b1]=a[i][b2];
           a[i][b2]=b3;
          }
        }
}
int main()
{
    ifstream ifs("input.txt");
    outfile.open("output.txt");
    if(!ifs)
    {
        cout<<"打开文件失败！"<<endl;
    }
    int l = 0;
    while (ifs>>times[l])
    {
        l++;
    }
    ifs.close();  //读入已知的加工时间；
    for (int i=0; times[i]!=0; i++)
    {
        cout<<times[i]<<"  ";
    }
    cout<<endl;
    for (int i=0; i<workpiecesnumber; i++)
    {

        for(int j=0; j<ordernumber; j++)
        {
            usetime[i][j]=times[ordernumber*i+j];
            cout<<usetime[i][j]<<"  ";
        }

        cout<<endl;
    }
    cout<<"//////////////////////////////////////////////////"<<endl;;
    srand(time(NULL));
    initialization();    //初始化种群；
                for(int g=0; g<G; g++)
                {
                    for(int c=0; c<populationnumber; c++)//计算每个个体适应度并存在ttime中；
                    {
                        fitness(c);
                        ttime[c]=makespan;
                    }
                    select();     //选择操作；
                    crossover();  //交叉操作；
                    mutation();   //变异操作；
                }

                    int flg8=ttime[0];
                    int flg9=0;
                    for(int c=0; c<populationnumber-1; c++)  //计算最后一代每个个体的适应度，并找出最优个体；
                    {

                        if(ttime[c]<flg8)
                        {
                            flg8=ttime[c];
                            flg9=c;
                        }
                    }
                gant(flg9);   //画出简易的流程图；
    outfile.close();
    return 0;
}
```

运行后的结果（output.txt）如下：



{{<figure src="/images/GAresult.png" title="（运行结果）">}}



**(2)MATLAB代码实现[^5]：**

主函数：

```matlab
function main()

piecetime = [2 4 6; ...             % 设备加工时间

    4 9 2; 4 2 8; 9 56; 5 2 7; 9 4 3];

equsize = [2 2 2];                  % 每个工序设备数目

piecesize = size(piecetime, 1);     % 工件数目

prosize = size(piecetime, 2);       % 工序数目

popsize = 20;      % 种群规模

cr = 0.6;          % 交叉概率

mr = 0.05;         % 变异概率

maxgen = 100;      % 迭代次数

bestobjvalue = zeros(1, maxgen);

bestpop = zeros(maxgen, piecesize);

avgobjvalue = zeros(1, maxgen);

bestptr = cell(1, maxgen);

bestper = cell(1, maxgen);

pop = initpop(popsize, piecesize);

for gen = 1:maxgen

    [objvalue, ptr,per] = calobjvalue(pop, piecetime, equsize);

    [bobjvalue,bindex] = min(objvalue);

    bestobjvalue(1,gen) = bobjvalue;

    bestpop(gen, :) =pop(bindex, :);

    avgobjvalue(1,gen) = sum(objvalue) / popsize;

    bestptr{1, gen} =ptr{1, bindex};

    bestper{1, gen} =per{1, bindex};

    fitness= calfitness(objvalue);     % 计算适应度值

    pop =selection(pop, fitness);      % 选择

    pop =crossover(pop, cr);           % 交叉

    pop =mutation(pop, mr);            % 变异

end

[~, finalindex] = min(bestobjvalue);

finalptr = bestptr{1, finalindex};

finalper = bestper{1, finalindex};

fprintf("最优序列:\n");

disp(bestpop(finalindex, :));

gantt = makegantt(finalptr, finalper, equsize);

figure(1);

imagesc(gantt);

colorbar;

title("加工流程图");

figure(2);

plot(1:maxgen, bestobjvalue);

title("最优时间变化图");

xlabel("代数"); ylabel("最优时间");

figure(3);

plot(1:maxgen, avgobjvalue);

title("平均时间变化图");

xlabel("代数"); ylabel("平均时间");

end
```

目标值函数：

```matlab
function [objvalue, ptr, per] = calobjvalue(pop, piecetime,equsize)

% 计算目标函数值

% pop          input  种群

% piecetime    input  工件加工时间

% equsize      input  每个工序设备数量

% objvalue     output 目标函数值（加工时间）

% ptr          output 工件加工时间记录，cell

% per          output 工件加工设备记录，cell

[popsize, piecesize] = size(pop);

prosize = size(equsize, 2);

objvalue = zeros(popsize, 1);

ptr = cell(1, popsize);

per = cell(1, popsize);

for i = 1:popsize

    pieceweight =pop(i, :);

    % 设备状态序列

    % [工序1设备1工序1设备2 工序2设备1 工序2设备2 ……]

    % 记录当前设备使用结束时间，默认为0表示未开始

    equstu = zeros(1,sum(equsize));

    % 对设备状态序列的工序分隔符

    % 大于等于当前设备最小值的索引是当前设备所处的工序

    % [2 35] 工序1有2台设备工序2有1台设备 工序3有2台设备

    prosep =cumsum(equsize);

    % 工件时间记录，记录每个工件每个工序的开始时间和结束时间

    % 行表示工件，相邻两列表示开始加工时间和停止加工时间

    % [1 2 2 3; 4 5 67]

    % 表示工件1第1工序加工时间为1-2，第2工序加工时间为2-3

    % 工件2第1工序加工时间为4-5，第2工序加工时间为6-7

    piecetimerecord =zeros(piecesize, prosize*2);

    % 工件设备记录，记录每个工件在工序中的加工设备

    % 行数表示工件，列表示该零件在每个工序加工设备

    % [1 2; 2 1]

    % 表示工件1在第1工序加工设备为1，第2工序加工设备为2

    % 工件2在第1工序加工设备为2，第2工序加工设备为1

    pieceequrecord =zeros(piecesize, prosize);

    % 对每一道工序

    %   如果是第1道工序，对工件按优先级排序

    %     其余工序上上一道工序完工时间对工件排序

    %   对排序后的每一件工件

    %     对该工序中可用机器按使用结束时间排序

    %     使用使用结束时间最小的机器

    %     加工开始时间为max{设备使用结束时间,零件上一工序完工时间}

    %     加工结束时间=加工开始时间+加工时间

    %     更新各个状态和记录矩阵

    for pro =1:prosize

        if(pro == 1)

            [~,piecelist] = sort(pieceweight);

        else

           tempendtime = piecetimerecord(:, (pro-1)*2);

           tempendtime = tempendtime';

            [~,piecelist] = sort(tempendtime);

        end

        for pieceindex= 1:length(piecelist)

            piece =piecelist(pieceindex);

           equtimelist = equstu(prosep(pro)-equsize(pro)+1:prosep(pro));

            [equtime,equlist] = sort(equtimelist);

            equ =equlist(1);

            if pro ==1

               piecestarttime = 0;

            else

               piecestarttime = piecetimerecord(piece, pro*2-2);

            end

            starttime= max(equtime(1), piecestarttime) + 1;

            endtime =starttime + piecetime(piece, pro) - 1;

           equstuindex = prosep(pro)-equsize(pro)+equ;

           equstu(equstuindex) = endtime;

           piecetimerecord(piece, pro*2-1) = starttime;

           piecetimerecord(piece, pro*2) = endtime;

           pieceequrecord(piece, pro) = equ;

        end

    end

    objvalue(i, 1) =max(max(piecetimerecord));

    ptr{1, i} =piecetimerecord;

    per{1, i} =pieceequrecord;

end

end
```

选择函数：

```matlab
function spop = selection(pop, fitness)

% 轮盘赌选择

% pop      input  种群

% fitness  input  适应度值

% spop     output 选择后生成的种群

[popsize, piecesize] = size(pop);

spop = zeros(popsize, piecesize);

sumfit = sum(fitness);

fitness = fitness ./ sumfit;

fitness = cumsum(fitness);

r = rand(1, popsize);

r = sort(r);

j = 1;

for i = 1:popsize

    while fitness(j)< r(i)

        j = j + 1;

    end

    spop(i, :) =pop(j, :);

end

% 由于上面轮盘赌方法特殊性，一个个体在相邻位置多次重复，故随机排序

rr = randperm(popsize);

spop(:, :) = spop(rr, :);

end
```

交叉函数：

```matlab
function cpop = crossover(pop, cr)

% 交叉

% pop      input  种群

% cr       input  交叉概率

% cpop     output 交叉后种群

[popsize, piecesize] = size(pop);

cpop = pop;

if mod(popsize,2) ~= 0

    nn = popsize - 1;

else

    nn = popsize;

end

% 父代father mother, 子代son daughter

% 在rl:ru中，son继承mother，daughter继承father

% 其余位置son继承father，daughter继承mother

for i = 1:2:nn

    if rand > cr

        continue;

    end

    [rl, ru] =makerlru(piecesize);

    father = pop(i, :);

    mother = pop(i+1, :);

    if father == mother

        continue;

    end

    son = zeros(1, piecesize);

    daughter = zeros(1,piecesize);

    son(rl:ru) = mother(rl:ru);

    daughter(rl:ru) =father(rl:ru);

    j = 1;

    for k = 1:piecesize

        if k >= rl &&k <= ru

            continue;

        end

        while ~isempty(find(son== father(j), 1))

            j = j + 1;

        end

        son(k) = father(j);

    end

    j = 1;

    for k = 1:piecesize

        if k >= rl &&k <= ru

            continue;

        end

        while~isempty(find(daughter == mother(j), 1))

            j = j + 1;

        end

        daughter(k) = mother(j);

    end

    cpop(i, :) = son;

    cpop(i+1, :) = daughter;

end

end
```

变异函数：

```matlab
function mpop = mutation(pop, mr)

% 变异，交换两个随即位置的基因

% pop      input  种群

% mr       input  变异概率

% mpop     output 变异后种群

[popsize, piecesize] = size(pop);

mpop = pop;

for i = 1:popsize

    if rand > mr

        continue;

    end

    r1 = randi(piecesize);

    r2 = randi(piecesize);

    temp  = mpop(i, r1);

    mpop(i, r1) = mpop(i, r2);

    mpop(i, r2) = temp;

end

end
```

初始化种群函数：

```matlab
function pop= initpop(popsize, piecesize)

%初始化种群

%popsize           input  种群规模

%piecesize         input  工件数量

%pop               output 种群

pop =zeros(popsize, piecesize);

for i =1:popsize

    pop(i, :) = randperm(piecesize);

end

end
```

计算适应度函数：

```matlab
functionfitness = calfitness(objvalue)

%计算适应度值

%objvalue      input  目标函数值

%fitness       output 适应度值

fitness = 1./ objvalue;

end
```

制作加工流程矩阵函数：

```matlab
functiongantt = makegantt(ptr, per, equsize)

%制作加工流程矩阵

%ptr       input  工件时间记录

%per       input  工件设备记录

%equsize   input  工序设备数量

%gantt     output 加工流程矩阵

finaltime =max(max(ptr));

[piecesize,prosize] = size(per);

cumsumequ =cumsum(equsize);

gantt =zeros(sum(equsize), finaltime);

for pro =1:prosize

    for i = 1:piecesize

        if pro == 1

            equ = per(i, pro);

        else

            equ = cumsumequ(pro - 1) + per(i,pro);

        end

        starttime = ptr(i, pro*2-1);

        endtime = ptr(i, pro*2);

        gantt(equ, starttime:endtime) = i; 

    end

end

end
```

生成随机数函数：

```matlab
function [rl,ru] = makerlru(maxnum)

%制作两个随机整数， rl < ru

%maxnum        input  最大数

%rl            output 小随机数

%ru            output 大随机数

r1 =randi(maxnum);

r2  =randi(maxnum);

while r2 ==r1

    r2 = randi(maxnum);

end

rl = min([r1,r2]);

ru = max([r1,r2]);

end
```

### 2.3 TSP问题(货郎担问题)[^6]

**问题定义：巡回旅行商问题**

给定一组n个城市和俩俩之间的直达距离，寻找一条闭合的旅程，使得每个城市刚好经过一次且总的旅行距离最短。

TSP问题也称为货郎担问题，是一个古老的问题。最早可以追溯到1759年Euler提出的骑士旅行的问题。1948年，由美国兰德公司推动，TSP成为近代「组合优化」领域的典型难题，也是典型的[「$$NP$$问题」]([(6条消息) 组合优化中的P问题，NP问题，NP-complete问题和NP-hard问题_m0_37407587的博客-CSDN博客](https://blog.csdn.net/m0_37407587/article/details/87169128) )。

TSP是一个具有广泛的应用背景和重要理论价值的组合优化问题。 近年来，有很多解决该问题的较为有效的算法不断被推出，例如Hopfield神经网络方法，模拟退火方法以及遗传算法方法等。

TSP搜索空间随着城市数n的增加而增大,所有的旅程路线组合数为$$(n-1)!\over2$$。在如此庞大的搜索空间中寻求最优解，对于常规方法和现有的计算工具而言，存在着诸多计算困难。故需借助遗传算法的搜索能力解决TSP问题。


[^1]: 引自百度百科
[^2]: 王铁方．计算机基因学 基于家族基因的网格信任模型[M]．北京：知识产权出版社，2016：93-94 
[^3]: 引自zhuanlan.zhihu.com/p/100337680
[^4]: 引自cloud.tencent.com/developer/article/1376883?from=article.detail.1522910
[^5]: 引自cloud.tencent.com/developer/article/1377357?from=article.detail.1522910
[^6]: 引自cloud.tencent.com/developer/article/1827527?from=article.detail.1830041
