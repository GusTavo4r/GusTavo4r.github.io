# (续2)打分式评价(TOPSIS,ENTROPY)


{{< admonition type=tip title="这是一个提示" open=false >}}

若(部分)数学公式、表达式或图表显示不完整、不正常，请保持网络流畅并刷新等待，加载时间可能较长~

<a href="mailto:gzzyyxy@gmail.com">任何疑问请致邮：E-mail</a>

{{< /admonition >}}

### 1.7 理想解方法（TOPSIS）

#### 1.7.1简述

TOPSIS法是一种理想目标相似性的顺序选优技术，常在多目标决策分析中使用。它通过归一化后的数据规范化矩阵，找出多个目标中的最优目标（理想解和反理想解），分别计算各评价目标与理想解和反理想解的距离，得到各目标与理想解的贴近度，并按贴近度进行大小排序，其反应了优劣水平。评价最好的对象应当是与最优目标距离最近，与最劣目标距离最远，距离的计算可采用明考斯基距离，常用的欧氏距离是明考斯基距离的特殊情况。

术语：

1. 理想解（positive ideal solution）：设想的最优解，其各个属性值为各备选方案中的最优值；
2. 负理想解（negative ideal solution）：和理想解相反。

#### 1.7.2评价步骤[^1]

设有$$m$$个评价对象$$D_1,D_2,\cdots,D_m$$，每个对象有$$n$$项评价指标$$X_1,X_2,\cdots,X_n$$，将其表示为矩阵形式：（各项$$x$$的值来自客观测量如pH值或主观评价如专家打分）


$$
D=\left[\begin{matrix}D_1\\\vdots\\D_i\\\vdots\\D_m \end{matrix}\right]=\left[ \begin{matrix}x_{11}&\cdots&x_{1j}&\cdots&x_{1n}\\\vdots&&\vdots&&\vdots\\x_{i1}&\cdots&x_{ij}&\cdots&x_{in}\\\vdots&&\vdots&&\vdots\\x_{m1}&\cdots&x_{mj}&\cdots&x_{mn} \end{matrix} \right]=\left[\begin{matrix}X_1&\cdots&X_i&\cdots&X_n \end{matrix}\right]\tag{1}
$$




**（1）数据正向化处理**

|      指标名称      |     指标特点     |
| :----------------: | :--------------: |
| 极大型(效益型)指标 |     越大越好     |
| 极小型(成本型)指标 |     越小越好     |
|     中间型指标     | 越接近某个值越好 |
|     区间型指标     | 落在某个区间最好 |

a.极小型指标正向化

$$\hat{x_i}=max-x_i$$ 或者 $$\hat{x_i}={1\over x_i}(x_i\gt0)$$

b.中间型指标正向化

设最佳值为$$x_m$$，取 $$M=max\{|x_i-x_m|\}$$，则 $$\hat{x_i}=1-{ {x_i-x_m}\over M}$$

c.区间型指标正向化

设最佳区间为$$[a,b]$$，取 $$M=max\{a-min\{x_i\},max\{x_i\}-b\}$$，则


$$
\hat{x_i}=\begin{aligned}\begin{cases}1-{ {a-x_i}\over M}&x_i\lt a\\1&a\lt x_i\lt b\\1-{ {x_i-b}\over M}&x\gt b \end{cases}\end{aligned}
$$


**（2）数据标准化(规范化)处理**

对（1）式矩阵中的列向量进行标准化处理：


$$
r_{ij}={x_{ij}\over \sqrt{\sum_{i=1}^mx_{ij}^2}}\tag{2}\quad \quad \quad i=1,2,\cdots,m;j=1,2,\cdots,n
$$


标准化处理后，各评价指标的值的平方和为1，适合TOPSIS法中计算欧氏距离的场合。

**（3）构造权重标准化矩阵**

设标准化矩阵为$$D$$，给定各个指标$$X_j$$的权重$$w_j$$构成权重向量$$W=(w_1,w_2,\cdots,w_n)$$，则


$$
v_{ij}=w_jr_{ij},i=1,2,\cdots m,j=1,2,\cdots n\tag{3}
$$


建立关于权重标准化值$$v_{ij}$$的权重标准化矩阵，记为$$D$$

**（4）确定理想解与负理想解**

经过了正向化和标准化处理的评分矩阵$$D$$，其数据均为极大型。取出每个指标（矩阵列向量）中的最大值，构成理想解向量，反之构成负理想解向量：


$$
\begin{aligned}
&A^+=\left[\begin{matrix}a_1^+,a_2^+,\cdots,a_n^+ \end{matrix}\right]=\left[\begin{matrix}max\{v_{11},v_{21},\cdots,v_{m1} \},max\{v_{12},v_{22},\cdots,v_{m2} \},\cdots,max\{v_{1n},v_{2n},\cdots,v_{mn}\} \end{matrix}\right]\\&A^-=\left[\begin{matrix}a_1^-,a_2^-,\cdots,a_n^- \end{matrix}\right]=\left[\begin{matrix}min\{v_{11},v_{21},\cdots,v_{m1} \},min\{v_{12},v_{22},\cdots,v_{m2} \},\cdots,min\{v_{1n},v_{2n},\cdots,v_{mn}\} \end{matrix}\right]
\end{aligned}\tag{4}
$$




**（5）计算欧氏距离**

计算每个评价对象的各指标值到理想解和负理想解的欧氏距离：


$$
S^+=\sqrt{\sum_{j=1}^n(v_{ij}-a_j^+)},S^-=\sqrt{\sum_{j=1}^n(v_{ij}-a_j^-)}\quad i=1,2,\cdots,m\tag{5}
$$


**（6）计算评价得分(理想解的贴近程度)**

定义对象$$D_i$$的得分为$$C_i$$


$$
C_i={S^+_i\over S^+_i+S^-_i},i=1,2,\cdots,m\tag{6}
$$


**（6）排序比较得出评价结果**

将上述所得得分进行从高到低排序，最后所得分数较高者，评价较好。若$$C_i=1$$，则其为最优目标，反之为最劣目标。

#### 1.7.3程序实现

**（1）MATLAB实现**

TOPSIS.m程序

```matlab
clear all
clc
%%  导入数据
% （1）在工作区右键，点击新建（Ctrl+N)，输入变量名称为X
% （2）双击进入X，输入或拷贝数据到X
% （3）关掉这个窗口，点击X变量，右键另存为，保存为mat文件
% （4）注意，代码和数据要放在同一个目录下哦，且Matlab的当前文件夹也要是这个目录。
load data_water_quality.mat

%%  数据预处理_正向化
[n,m] = size(X);
disp(['共有' num2str(n) '个评价对象, ' num2str(m) '个评价指标']) 
Judge = input(['这' num2str(m) '个指标是否需要经过正向化处理，需要请输入1 ，不需要输入0：  ']);

if Judge == 1
    Position = input('请输入需要正向化处理的指标所在的列，例如[2,3,6]： '); %[2,3,4]
    disp('请输入需要处理的这些列的指标类型（1：极小型， 2：中间型， 3：区间型） ')
    Type = input('例如[1,3,2]：  '); %[2,1,3]
    % 注意，Position和Type是两个同维度的行向量
    for i = 1 : size(Position,2)%对每一列进行正向化处理
        X(:,Position(i)) = Positivization(X(:,Position(i)),Type(i),Position(i));
    % 第一个参数是要正向化处理的那一列向量 X(:,Position(i))
    % 第二个参数是对应的这一列的指标类型（1：极小型， 2：中间型， 3：区间型）
    % 第三个参数是告诉函数我们正在处理的是原始矩阵中的哪一列
    % 返回值返回正向化之后的指标
    end
    disp('正向化后的矩阵 X =  ')
    disp(X)
end

%% 数据预处理_标准化
Z = X ./ repmat(sum(X.*X) .^ 0.5, n, 1);
disp('标准化矩阵 Z = ')
disp(Z)

%% 指标权重赋值
disp("请输入是否需要增加权重向量，需要输入1，不需要输入0")
Judge = input('请输入是否需要增加权重： ');
if Judge == 1
    disp(['有多少个指标就输入多少个权重数(权重和为1)，如[0.25,0.25,0.5]']);
    weigh = input(['请输入输入' num2str(m) '个权重: ']);
        if abs(sum(weigh) - 1)<0.000001 && size(weigh,1) == 1 && size(weigh,2) == m   % 这里要注意浮点数的运算是不精准的。
        else
            weigh = input('你输入的有误，请重新输入权重行向量: ');
        end
else
    weigh = ones(1,m) ./ m ; %如果不需要加权重就默认权重都相同，即都为1/m
end

%% 计算与最大值的距离和最小值的距离，并算出得分
D_P = sum([(Z - repmat(max(Z),n,1)) .^ 2 ] .* repmat(weigh,n,1) ,2) .^ 0.5;   % D+ 与最大值的距离向量
D_N = sum([(Z - repmat(min(Z),n,1)) .^ 2 ] .* repmat(weigh,n,1) ,2) .^ 0.5;   % D- 与最小值的距离向量
S = D_N ./ (D_P+D_N);    % 未归一化的得分
disp('最后的得分为：')
stand_S = S / sum(S)% 归一化的得分
[sorted_S,index] = sort(stand_S ,'descend')%对得分进行排序并返回原来的位置
plot(sorted_S,'r-o')
xmin=1;xmax = size(sorted_S,1);
ymin = 0;ymax = max(sorted_S)+min(sorted_S);
axis([xmin xmax ymin ymax]); % 设置坐标轴在指定的区间
grid on
xlabel('方案');ylabel('分数');%坐标轴表示对bai象标签
title('TOPSIS算法最终评分排序')
```

Positivization.m程序

```matlab
function [posit_x] = Positivization(x,type,i)
% 输入变量有三个：
% x：需要正向化处理的指标对应的原始列向量
% type： 指标的类型（1：极小型， 2：中间型， 3：区间型）
% i: 正在处理的是原始矩阵中的哪一列
% 输出变量posit_x表示：正向化后的列向量
    if type == 1  %极小型
        disp(['第' num2str(i) '列是极小型，正在正向化'] )
        posit_x = Min2Max(x);  %调用Min2Max函数来正向化
        disp(['第' num2str(i) '列极小型正向化处理完成'] )
        disp('~~~~~~~~~~~~~~~~~~~~分界线~~~~~~~~~~~~~~~~~~~~')
    elseif type == 2  %中间型
        disp(['第' num2str(i) '列是中间型'] )
        best = input('请输入最佳的那一个值(中间的那个值)： ');
        posit_x = Mid2Max(x,best);
        disp(['第' num2str(i) '列中间型正向化处理完成'] )
        disp('~~~~~~~~~~~~~~~~~~~~分界线~~~~~~~~~~~~~~~~~~~~')
    elseif type == 3  %区间型
        disp(['第' num2str(i) '列是区间型'] )
        a = input('请输入区间的下界： ');
        b = input('请输入区间的上界： '); 
        posit_x = Inter2Max(x,a,b);
        disp(['第' num2str(i) '列区间型正向化处理完成'] )
        disp('~~~~~~~~~~~~~~~~~~~~分界线~~~~~~~~~~~~~~~~~~~~')
    else
        disp('没有这种类型的指标，请检查Type向量中是否有除了1、2、3之外的其他值')
    end
end
```

### 1.8 加权积（Weighted Average Method）

#### 1.8.1简述

加权平均法，利用过去若干个按照时间顺序排列起来的同一变量的观测值并以时间顺序变量出现的次数为权数，计算出观测值的加权算术平均数，以这一数字作为预测未来期间该变量预测值的一种趋势预测法。

加权平均法可根据本期期初结存存货的数量和金额与本期存入存货的数量和金额，在期末以此计算本期存货的加权平均单价，作为本期发出存货和期末结存存货的价格，一次性计算本期发出存货的实际成本。

#### 1.8.2计算方法


$$
\widetilde{x}={x_1w_1+x_2w_2+\cdots+x_nw_n\over w_1+w_2+\cdots+w_n},\sum_{i=1}^nw_i=1\tag{7}
$$




### 1.9 熵权法（EWM）

#### 1.9.1简述

熵权法的基本思路是根据指标变异性的大小来确定客观权重。一般来说，若某个指标的信息熵$E_j$越小，表明指标值得变异程度越大，提供的信息量越多，在综合评价中所能起到的作用也越大，其权重也就越大。相反，某个指标的信息熵$E_j$越大，表明指标值得变异程度越小，提供的信息量也越少，在综合评价中所起到的作用也越小，其权重也就越小。 

相对应的，由于该模型的本质是用有限个决策样本去“估计”指标的信息熵，在样本量过少的情况下，基于熵权法所计算得出的权重则有可能出现较大误差。一般来讲，样本决策数必须大于等于指标数。 

#### 1.9.2赋权步骤

**1.数据标准化**

将各个指标的数据进行标准化处理。假设给定了$k$个指标$X_1,X_2,\cdots,X_k$，其中$X_i=\{x_1,x_2,...x_n\}$。假设对各指标数据标准化后的值为$Y_1,Y_2,\cdots ,Y_k$，那么 ：


$$
Y_{ij}={X_{ij}-min{X_i}\over max{X_j}-minX_i}\tag{8}
$$


**2.求各指标的信息熵**

一组数据的信息熵：


$$
E_j=-ln^{-1}(n)\sum_{i=1}^np_{ij}lnp_{ij}\tag{9}
$$


其中$P_{ij}={Y_{ij}\over \sum_{i=1}^nY_{ij} }$，若$P_{ij}=0$，则定义$\underset{P_{ij}\rightarrow 0 }{lim}P_{ij}lnP_{ij}=0$

**3.确定指标权重**

根据信息论中信息熵的计算公式，计算出各个指标的信息熵为$E_1,E_2,\cdots,E_k$。

通过信息熵计算各指标的权重：


$$
W_i={1-E_i\over k-\sum E_i}\tag{10}
$$

#### 1.9.3实例及代码实现

MATLAB实例：

```matlab
function weights = EntropyWeight(R)
%% 熵权法求指标权重,R为输入矩阵,返回权重向量weights

[rows,cols]=size(R);   % 输入矩阵的大小,rows为对象个数，cols为指标个数
k=1/log(rows);         % 求k

f=zeros(rows,cols);    % 初始化fij
sumBycols=sum(R,1);    % 输入矩阵的每一列之和(结果为一个1*cols的行向量)
% 计算fij
for i=1:rows
    for j=1:cols
        f(i,j)=R(i,j)./sumBycols(1,j);
    end
end

lnfij=zeros(rows,cols);  % 初始化lnfij
% 计算lnfij
for i=1:rows
    for j=1:cols
        if f(i,j)==0            lnfij(i,j)=0;
        else
            lnfij(i,j)=log(f(i,j));
        end
    end
end

Hj=-k*(sum(f.*lnfij,1)); % 计算熵值Hj
weights=(1-Hj)/(cols-sum(Hj));
end
```

### 1.10 信息熵法

#### 1.10.1简述

信息熵这个词是C.E.Shannon（香农）从热力学中借用过来的。热力学中的热熵是表示分子状态混乱程度的物理量，香农用信息熵的概念来描述信源的不确定度。 信息熵是一种用于衡量系统内部信息量的度量。在信息论中，信息是系统有序程度的一种度量。信息是确定性的增加，不确定性的减少（香农定义）。而信息熵是系统无序程度的一种度量，是「系统不确定性」的量度。两者绝对值相等，但符号相反。一个系统的信息熵越小，该系统所含的信息量越大。

信息熵被广泛用于计算机编码，通信理论，博弈论等与“信息量”和 “不确定性”相关的理论模型中。

熵权法就是一个通过信息熵理论确定系统中各指标权值的赋值方法，能够较为精确客观地判断系统中各指标对总评价的贡献大小，是信息熵理论在数学建模领域中最重要的应用之一。

#### 1.10.2描述过程

通常一个信源发出什么符号是不确定的，衡量它可以根据其出现的概率来度量。概率大，出现机会多，不确定性小；反之不确定性就大。 不确定性函数f是概率P的「减函数」；两个独立符号所产生的不确定性应等于各自不确定性之和，即 $$f(P_1,P_2)=f(P1_)+f(P_2)$$ ，这称为「可加性」。同时满足这两个条件的函数 $$f$$ 是对数函数，即


$$
f(P)=log{1\over P}=-logP\tag{11}
$$
在信源中，考虑的不是某一单个符号发生的不确定性，而是要考虑这个信源所有可能发生情况的平均不确定性。若信源符号有n种取值：$$U_1,U_2,\cdots,U_n$$ ，对应概率为：$$P_1,P_2,\cdots,P_n$$ ，且各种符号的出现彼此独立。这时，信源的平均不确定性应当为单个符号不确定性 $$-logP_i$$ 的统计平均值 $$E$$ ，称为「信息熵」[^2]，即



$$
H(X)=E(-logP_i)\tag{12}
$$



一个离散型随机变量 $$X$$ 的信息熵 $$H(X)$$ 定义为：



$$
H(X)=-\sum P(x_i)\log P(x_i)\tag{13}
$$



式中对数一般取2为底，单位为比特。其特点：

- 单调性，即发生概率越高的事件，其所携带的信息熵越低；
- 非负性，即收到一个信源符号所获得的信息量应为正值，H(U)≥0 ； 
- 累加性，即多随机事件同时发生存在的总不确定性的量度是可以表示为各事件不确定性的量度的和。

若事件 $$X=A$$ ，事件 $$Y=B$$ 同时发生，且有：


$$
P(X=A,Y=B)=P(X=A)\times P(Y=B)
$$


即两个事件相互独立，则有信息熵 $$H(A,B)=H(A)+H(B)$$ ；若两事件不相互独立，则有 $$H(A,B)=H(A)+H(B)-I(A,B)$$ ，$$I(A,B)$$ 为互信息（mutual Information），代表一个随机变量包含另一个随机变量信息量的度量。 

#### 1.10.3应用

##### 1.10.3.1熵权法

##### 1.10.3.2霍夫曼编码

##### 1.10.3.3条件熵与信息增益

##### 1.10.3.4交叉熵与相对熵

[^1]: [TOPSIS(逼近理想解)算法原理详解与代码实现 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/266689519) 
[^2]: 引自百度百科
