# 傅里叶变换推导


## 1.傅里叶级数

### 1.1三角函数的正交性

给出一个三角函数系：$$(0), 1, sinx, cosx, sin2x, cos2x, \cdots, sinnx, cosnx$$ ，那么他们满足关系


$$
\int_{-\pi}^{\pi}sinnxsinmxdx=0 \quad n\neq m\\\tag{1}
\int_{-\pi}^{\pi}sinnxcosmxdx=0 \quad n\neq m\\
\int_{-\pi}^{\pi}sin^2nxdx=\int_{-\pi}^{\pi}cos^2nxdx=\pi
$$


即为三角函数系的正交性。（利用积化和差可证）

### 1.2周期为 $$2\pi$$ 的函数的傅里叶级数展开

设 $$f(t)$$ 为以 $$2\pi$$ 为周期的实函数，将其用三角函数的组合式逼近，容易想到设其为


$$
f(x)=\sum_{n=0}^{+\infty}a_ncosnx+\sum_{n=0}^{+\infty}b_nsinnx \tag{2}
$$


一般将其写为


$$
f(x)={a_0\over2}+\sum_{n=1}^{+\infty}(a_ncosnx+b_nsinnx)\tag{3}
$$


(2)式和(3)式本质上相同，由于


$$
f(x)=\sum_{n=0}^{+\infty}a_ncosnx+\sum_{n=0}^{+\infty}b_nsinnx=a_0+\sum_{n=1}^{+\infty}(a_ncosnx+b_nsinnx)\tag{3}
$$


**(1)考察 $$a_0$$ 的表达式**，对 $$f(x)$$ 在 $$[-\pi,\pi]$$ 积分


$$
\begin{aligned}
\int_{-\pi}^\pi f(x)dx&=\int_{-\pi}^\pi a_0dx+\int_{-\pi}^\pi \sum_{n=1}^{+\infty}a_ncosnxdx+\int_{-\pi}^\pi \sum_{n=1}^{+\infty}b_nsinnxdx\\
&=2\pi a_0+a_n\sum_{n=1}^{+\infty}(\int_{-\pi}^\pi 1\cdot cosnxdx)+b_n\sum_{n=1}^{+\infty}(\int_{-\pi}^\pi 1\cdot sinnxdx)\\
&=2\pi a_0(正交性)\\
\end{aligned}
$$


则有


$$
a_0 = {1\over2\pi}\int_{-\pi}^\pi f(x)dx\\let\quad{a_0'\over2}={1\over2\pi}\int_{-\pi}^\pi f(x)dx\\then\quad a_0'={1\over\pi}\int_{-\pi}^\pi f(x)dx\tag{4}
$$


**(2)考察 $$a_n$$ 的表达式**

观察到包含 $$a_n$$系数的项为余弦项，考虑到三角函数的正交性，有


$$
\begin{aligned}
\int_{-\pi}^\pi f(x)cosmxdx&=\int_{-\pi}^\pi {a_0\over2}cosmxdx+\int_{-\pi}^\pi \sum_{n=1}^{+\infty}a_ncosnxcosmxdx+\int_{-\pi}^\pi \sum_{n=1}^{+\infty}b_nsinnxcosmxdx\\
&=\pi a_n
\end{aligned}
$$


因此


$$
a_n={1\over\pi}\int_{-\pi}^\pi f(x)cosnxdx\tag{5}
$$
**(3)考察 $$b_n$$ 表达式**

同理有


$$
\begin{aligned}
\int_{-\pi}^\pi f(x)sinmxdx&=\int_{-\pi}^\pi {a_0\over2}sinmxdx+\int_{-\pi}^\pi \sum_{n=1}^{+\infty}a_ncosnxsinmxdx+\int_{-\pi}^\pi \sum_{n=1}^{+\infty}b_nsinnxsinmxdx\\
&=\pi b_n
\end{aligned}
$$


所以


$$
b_n={1\over\pi}\int_{-\pi}^\pi f(x)sinnxdx\tag{6}
$$


综上所述，周期为 $$2\pi$$ 的实函数的傅里叶级数展开为


$$
f(x)={a_0\over2}+\sum_{n=1}^{+\infty}(a_ncosnx+b_nsinnx)\\\tag{7}
a_0={1\over\pi}\int_{-\pi}^\pi f(x)dx\\
a_n={1\over\pi}\int_{-\pi}^\pi f(x)cosnxdx\\
b_n={1\over\pi}\int_{-\pi}^\pi f(x)sinnxdx\\
$$


### 1.3周期为T的函数的傅里叶级数展开

设 $$f(t)$$ 为以 $$T(0\leq T\leq+ \infty )$$ 为周期的实函数，且在 $$[-{T\over 2},{T\over 2}]$$ 满足 Dirichlet 条件，则 $$f_T(t)$$ 可展开为傅里叶级数，设在连续点 $$t$$ 处

作变换


$$
x={\pi\over T/2}t={2\pi\over T}t\triangleq wt \quad (w={2\pi\over T})
$$


代入到(7)式中，即得

$$
f_T(t)={a_0\over2}+\sum_{n=1}^{+\infty}(a_ncosnwt+b_nsinnwt)\\\tag{8}
a_n={2\over T}\int_{-{T\over 2} }^{T\over 2} f(t)cosnwtdx\quad n=0,1,2,\cdots\\
b_n={2\over T}\int_{-{T\over 2} }^{T\over 2} f(t)sinnwtdx\quad n=1,2,3,\cdots\\
$$

在间断点处满足


$$
{f_T(t+0)+f_T(t-0)\over 2}={a_0\over2}+\sum_{n=1}^{+\infty}(a_ncosnwt+b_nsinnwt)\tag{9}
$$


### 1.4傅里叶级数展开式的复数形式

将三角函数式的复数式代入式(8)得




$$
\begin{aligned}
f_T(t)&={a_0\over2}+\sum_{n=1}^{+\infty}(a_n{e^{inwt}+e^{-inwt}\over2}+b_n{e^{inwt}-e^{-inwt}\over2i})\\
&={a_0\over2}+\sum_{n=1}^{+\infty}({ {a_n-ib_n}\over2}e^{inwt})+\sum_{n=1}^{+\infty}({ {a_n+ib_n}\over2}e^{-inwt})\\
&={a_0\over2}+\sum_{n=1}^{+\infty}({ {a_n-ib_n}\over2}e^{inwt})+\sum_{n=-\infty}^{-1}({ {a_{-n}+ib_{-n} }\over2}e^{inwt})\\
&=\sum_{n=0}^0{a_0\over2}e^{inwt}+\sum_{n=1}^{+\infty}({ {a_n-ib_n}\over2}e^{inwt})+\sum_{n=-\infty}^{-1}({ {a_{-n}+ib_{-n} }\over2}e^{inwt})\\
&\triangleq\sum_{n=-\infty}^{+\infty}c_ne^{inwt}
\end{aligned}\tag{10}
$$
其中系数项 $$c_n$$


$$
c_n=
\begin{cases}
a_0\over2&n=0\\
a_n-ib_n\over2&n=1,2,3,\cdots\\
a_n+ib_n\over2&n=-1,-2,-3,\cdots
\end{cases}\tag{11}
$$
具体考察 $$c_n$$ 表达式


$$

$$

## 附 常见的傅里叶变换对

1. 阶跃信号

   
   $$
   单位阶跃信号：u(t)\rightarrow {1\over iw}+\pi\delta(w)\\
   单位斜坡信号：tu(t)\rightarrow -{1\over w^2}+i\pi\delta'(w)\quad(i{d\over dw}[{1\over iw}+\pi\delta(w)])
   $$
   

   $$u(t)$$ 是一个积分器，$$\int_{-\infty}^tf(s)ds=f(t)*u(t)$$。

2. 冲激信号

   
   $$
   单位冲激信号：\delta(t)\rightarrow 1,\quad \delta(t-t_0)\rightarrow e^{-iwt_0}\\
   k阶导数：\delta^{(n)}(t)\rightarrow (iw)^n
   $$
   

3. 直流信号

   
   $$
   1 \to 2\pi\delta(w)\\
   t^n \to 2\pi i^{n} \delta^{(n)}(w)
   $$
   

4. 指数信号

   
   $$
   \begin{aligned}
   单边指数信号：& e^{-\beta t}u(t) \to {1\over\beta+iw},\quad e^{\beta t}u(-t) \to {1\over \beta-iw}\\
   双边指数信号：&e^{-\beta|t|}\to {2\beta\over \beta^2+w^2}\quad(e^{-\beta|t|}=e^{-\beta t}u(t)+e^{\beta t}u(-t))\\
   &e^{-\beta t}u(t)+e^{\beta t}u(-t) \to {-2iw\over \beta^2+w^2}\\
   虚指数信号：&e^{iw_0t}\to 2\pi\delta(w-w_0),\quad e^{-iw_0t}\to 2\pi\delta(w+w_0)\\
   三角信号：&sin(w_0t)\to i\pi(\delta(w+w_0)-\delta(w-w_0))\\
   &cos(w_0t)\to \pi(\delta(w+w_0)+\delta(w-w_0))\\
   调频信号：&f(t)sin(w_0t)\to{i\over2}[F(w+w_0)-F(w-w_0)]\\
   &f(t)cos(w_0t)\to{1\over2}[F(w+w_0)+F(w-w_0)]\\
   指数调频信号：&e^{-\beta t}sin(w_0t)u(t)\to{w_0\over(\beta+iw)^2+w_0^2}\\
   &e^{-\beta t}cos(w_0t)u(t)\to{\beta+iw\over(\beta+iw)^2+w_0^2}
   \end{aligned}
   $$
   


