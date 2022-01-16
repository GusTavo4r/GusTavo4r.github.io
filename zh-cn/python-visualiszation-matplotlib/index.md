# PYTHON可视化-MATPLOTLIB


> 官方文档：[Matplotlib: Python plotting — Matplotlib 3.4.3 documentation](https://matplotlib.org/) 

- pip安装matplotlib：

  ```powershell
   python -m install -U pip
   python -m install -U matplotlib
  ```

## Matplotlib.Pyplot

- 使用时导入：

  ```python
  import matplotlib.pyplot as plt
  ```

- [Plot Gallery](https://matplotlib.org/stable/gallery/index.html#gallery) 
- [Matplotlib](https://www.runoob.com/matplotlib/matplotlib-tutorial.html)

常用的几个图表：

### 信号相干图(coherence of signals)

```python
import numpy as np
import matplotlib.pyplot as plt

# Fixing random state for reproducibility
np.random.seed(19680801)

dt = 0.01
t = np.arange(0, 30, dt)
nse1 = np.random.randn(len(t))                 # white noise 1
nse2 = np.random.randn(len(t))                 # white noise 2

# Two signals with a coherent part at 10Hz and a random part
s1 = np.sin(2 * np.pi * 10 * t) + nse1
s2 = np.cos(2 * np.pi * 10 * t) + nse2

fig, axs = plt.subplots(2, 1)                  #创建2×1的画布，两个图
axs[0].plot(t, s1, t, s2)                      #第一个图
axs[0].set_xlim(0, 2)                          #set_xlim设定x轴范围
axs[0].set_xlabel('time')
axs[0].set_ylabel('s1 and s2')
axs[0].grid(True)                              #显示网格线

cxy, f = axs[1].cohere(s1, s2, 256, 1. / dt)   #第二个图，cxy为返回值相干矢量，f为返回值频率
axs[1].set_ylabel('coherence')

fig.tight_layout()                             #自动调整子图位置
plt.show()
```

建立画布：

```python
matplotlib.pyplot.subplots(nrows=1, ncols=1, *, sharex=False, sharey=False, squeeze=True, subplot_kw=None, gridspec_kw=None, **fig_kw)
```

计算两列数据的相干性：

```python
cxy, frequency = axes.cohere(self, x, y, NFFT=256, Fs=2, Fc=0, detrend=, window=, noverlap=0, pad_to=None, sides=’default’, scale_by_freq=None, *, data=None, **kwargs)
```

### 信号的交叉谱密度变化图(the cross spectral density of  signals)

```python
import numpy as np
import matplotlib.pyplot as plt


fig, (ax1, ax2) = plt.subplots(2, 1)
# make a little extra space between the subplots
fig.subplots_adjust(hspace=0.5)

dt = 0.01
t = np.arange(0, 30, dt)

# Fixing random state for reproducibility
np.random.seed(19680801)


nse1 = np.random.randn(len(t))                 # white noise 1
nse2 = np.random.randn(len(t))                 # white noise 2
r = np.exp(-t / 0.05)

cnse1 = np.convolve(nse1, r, mode='same') * dt   # colored noise 1
cnse2 = np.convolve(nse2, r, mode='same') * dt   # colored noise 2
#np.convolve: Returns the discrete, linear convolution of two one-dimensional sequences.

# two signals with a coherent part and a random part
s1 = 0.01 * np.sin(2 * np.pi * 10 * t) + cnse1
s2 = 0.01 * np.sin(2 * np.pi * 10 * t) + cnse2

ax1.plot(t, s1, t, s2)
ax1.set_xlim(0, 5)
ax1.set_xlabel('time')
ax1.set_ylabel('s1 and s2')
ax1.grid(True)

cxy, f = ax2.csd(s1, s2, 256, 1. / dt)
ax2.set_ylabel('CSD (db)')
plt.show()
```

[计算两列信号的交叉谱密度：](https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.csd.html#matplotlib.axes.Axes.csd)

```python
Axes.csd(x, y, NFFT=None, Fs=None, Fc=None, detrend=None, window=None, noverlap=None, pad_to=None, sides=None, scale_by_freq=None, return_line=None, *, data=None, **kwargs)[source]
```

### [条形图标记(Bar chart label)]([Bar Label Demo — Matplotlib 3.4.3 documentation](https://matplotlib.org/stable/gallery/lines_bars_and_markers/bar_label_demo.html#sphx-glr-gallery-lines-bars-and-markers-bar-label-demo-py) )

```python
Axes.bar(x, height, width=0.8, bottom=None, *, align='center', data=None, **kwargs)
```

### 堆积条形图(Stacked bar chart)

```python
import matplotlib.pyplot as plt

labels = ['G1', 'G2', 'G3', 'G4', 'G5']
men_means = [20, 35, 30, 35, 27]			#data1
women_means = [25, 32, 34, 20, 25]			#data2
men_std = [2, 3, 4, 1, 2]
women_std = [3, 5, 2, 3, 3]
width = 0.35       # the width of the bars: can also be len(x) sequence

fig, ax = plt.subplots()

ax.bar(labels, men_means, width, yerr=men_std, label='Men')
ax.bar(labels, women_means, width, yerr=women_std, bottom=men_means,
       label='Women')

ax.set_ylabel('Scores')
ax.set_title('Scores by group and gender')
ax.legend()

plt.show()
```

### [误差条子采样(Errorbar subsampling)](https://matplotlib.org/stable/gallery/lines_bars_and_markers/errorbar_subsample.html#sphx-glr-gallery-lines-bars-and-markers-errorbar-subsample-py)

### 离散分布水平条形图(Discrete distribution as horizontal bar chart)

```python
import numpy as np
import matplotlib.pyplot as plt


category_names = ['Strongly disagree', 'Disagree',
                  'Neither agree nor disagree', 'Agree', 'Strongly agree']
results = {
    'Question 1': [10, 15, 17, 32, 26],
    'Question 2': [26, 22, 29, 10, 13],
    'Question 3': [35, 37, 7, 2, 19],
    'Question 4': [32, 11, 9, 15, 33],
    'Question 5': [21, 29, 5, 5, 40],
    'Question 6': [8, 19, 5, 30, 38]
}


def survey(results, category_names):
    """
    Parameters
    ----------
    results : dict
        A mapping from question labels to a list of answers per category.
        It is assumed all lists contain the same number of entries and that
        it matches the length of *category_names*.
    category_names : list of str
        The category labels.
    """
    labels = list(results.keys())
    data = np.array(list(results.values()))
    data_cum = data.cumsum(axis=1)
    category_colors = plt.get_cmap('RdYlGn')(
        np.linspace(0.15, 0.85, data.shape[1]))

    fig, ax = plt.subplots(figsize=(9.2, 5))
    ax.invert_yaxis()
    ax.xaxis.set_visible(False)
    ax.set_xlim(0, np.sum(data, axis=1).max())

    for i, (colname, color) in enumerate(zip(category_names, category_colors)):
        widths = data[:, i]
        starts = data_cum[:, i] - widths
        rects = ax.barh(labels, widths, left=starts, height=0.5,
                        label=colname, color=color)

        r, g, b, _ = color
        text_color = 'white' if r * g * b < 0.5 else 'darkgrey'
        ax.bar_label(rects, label_type='center', color=text_color)
    ax.legend(ncol=len(category_names), bbox_to_anchor=(0, 1),
              loc='lower left', fontsize='small')

    return fig, ax


survey(results, category_names)
plt.show()
```

### 3D surface plot

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

X = np.arange(-5, 5, 0.25)
Y = np.arange(-5, 5, 0.25)
X, Y = np.meshgrid(X, Y)#Return coordinate matrices from coordinate vectors.
R = np.sqrt(X**2 + Y**2)
Z = np.sin(R)

fig = plt.figure()
ax = Axes3D(fig, auto_add_to_figure=False)
fig.add_axes(ax)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.viridis)

plt.show()
```

建立一个3D图：

```python
plot_surface(X, Y, Z, *args, norm=None, vmin=None, vmax=None, lightsource=None, **kwargs)[source]
```

```python
numpy.meshgrid(*xi, copy=True, sparse=False, indexing='xy')
```


