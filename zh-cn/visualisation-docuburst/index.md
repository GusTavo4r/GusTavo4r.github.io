# 可视化-文档散


## 1. 简述

### 1.1 基本介绍

在浏览互联网和大型文本数据库时，比较许多文档是一项常见任务。分析和比较冗长文档的内容是可视化分析中的一项重要任务。多年来，文档内容概述一直是信息可视化领域的一个活跃研究领域，但报告的作品并未在可视化中使用有价值的人类注释语言结构，而是使用预先选定的一组感兴趣的术语提供主题内容的细节。因此提供了一个基于人类注释IS-A关系的文档内容的交互式可视化，即WordNet的名词和动词层次结构。

文档散使用词汇库中的结构关系来布局关键词，同时使用词语关系网中具有上下语义关系的词语来布局关键词，从而揭示文本中的内容。上下语义关系是指词语之间往往存在语义层级的关系，也就是说，一些词语是某些词语的下义词。而在一篇文章中，具有上下语义关系的词语一般是同时存在的。

DocuBurst以放射状层次圆环的形式展示文本结构[^1]，上下文的层次关系基于Wordnet[^2]方法获得。其中，某单词的频率等于该单词子树中单词在文档中的频率之和，单一层次中某单词所覆盖的弧度表示其与这一层次中其他单词频率的比例关系。某个单词的子树可以根据这个单词所属的同义词集的个数进行颜色划分，每个同义词集都具有相同颜色，与之相反，某单词如果属于单一同义词集，则他的子树显示为单一色调[^1]。

### 1.2 适用情况

文本内容的可视化可以根据目标的不同分为两种：对于文本语义内容快速、粗略的可视化展示和对文本中或文本间模式的发现和特定可视化[^1]。以动车组运行故障文本的可视化研究为例，在模型的选择上要能够同时到达两个目标，首先能够概略的展示运行故障文本中的概要语义信息，又能够通过查询的交互方式查找特定类型故障、特定关键零部件故障在不同车型或不同系统中的分布特点，以便于将研究成果迅速应用到实际的动车组车辆运营维修管理中。通过对基于词频可视化和基于文本间关系可视化的研究，选择DocuBurst作为基础模型[^3]。

## 2. 软件实现

### 2.1 Wijmo

#### 2.1.1 引入Wijmo相关的样式和js文件

1、引入自定义的js文件

```HTML
<script src="scripts/DataLoader.js"></script>
<script src="scripts/app.js"></script>
```

2、定义一个DIV

```HTML
<div id="periodic-sunburst" class="periodic-sunburst"></div>
```

#### 2.1.2 DataLoader.js，获得数据

创建一个DataLoader类，其中提供两个方法。readFile方法读取json文件获得数据。isInclude 方法判断数组中是否存在指定的元素。generateCollectionView方法中对数据进行加工处理。

 ```JS
var DataLoader = {};
// 一级分类
var METALS_TITLE = "金属";
var NON_METALS_TITLE = "非金属";
var OTHERS_TITLE = "过渡元素";
// 二级分类
var METAL_TYPES = '碱金属|碱土金属|过渡金属|镧系元素|锕系元素|其他金属'.split('|');
var NON_METAL_TYPES = '惰性气体|卤素|非金属'.split('|');
var OTHER_TYPES = '准金属|超锕系'.split('|');
DataLoader = {
    readFile: function (filePath, callback) {
        var reqClient = new XMLHttpRequest();
        reqClient.onload = callback;
        reqClient.open("get", filePath, true);
        reqClient.send();
    },
    isInclude: function (arr, data) {
        if (arr.toString().indexOf(data) > -1)
            return true;
        else
            return false;
    },
    generateCollectionView: function (callback) {
        DataLoader.readFile('data/elements.json', function (e) {
            // 获取数据
            var rawElementData = JSON.parse(this.responseText);
            var elementData = rawElementData['periodic-table-elements'].map(function (item) {
                item.properties.value = 1;
                return item.properties;
            });
            var data = new wijmo.collections.CollectionView(elementData);
            //  利用wijmo.collections.PropertyGroupDescription 进行第一级分组
            data.groupDescriptions.push(new wijmo.collections.PropertyGroupDescription('type', function (item, prop) {
                if (DataLoader.isInclude(METAL_TYPES, item[prop])) {
                    return METALS_TITLE;
                } else if (DataLoader.isInclude(NON_METAL_TYPES, item[prop])) {
                    return NON_METALS_TITLE;
                } else {
                    return OTHERS_TITLE;
                }
            }));
            // 进行第二级分组
            data.groupDescriptions.push(new wijmo.collections.PropertyGroupDescription('type', function (item, prop) {
                return item[prop];
            }));
            callback(data);
        });
    }
};
 ```

generateCollectionView方法中调用readFile获得json数据，之后利用Wijmo中提供的CollectionView对数据进行2级分组。第1级是金属、非金属、过渡元素。第2级分别是他们的子级别。第3级是元素，每个元素的Value都是1，表示元素的占比相同。

#### 2.1.2 app.js，数据分组

和前边的简单示例相比，这里绑定的数据源是CollectionView.Groups，它是CollectionView中的第一级分组。

 ```JS
var mySunburst;
function setSunburst(elementCollectionView) {
    // 创建旭日图控件
    mySunburst = new wijmo.chart.hierarchical.Sunburst('#periodic-sunburst');
    mySunburst.beginUpdate();
    // 设置旭日图的图例不显示
    mySunburst.legend.position = 'None';
    // 设置内圆半径
    mySunburst.innerRadius = 0.1;
    // 设置选择模式
    mySunburst.selectionMode = 'Point';
    // 设置数据显示的位置
    mySunburst.dataLabel.position = 'Center';
    // 设置数据显示的内容
    mySunburst.dataLabel.content = '{name}';
    // 进行数据绑定
    mySunburst.itemsSource = elementCollectionView.groups;
    // 包含图表值的属性名
    mySunburst.binding = 'value';
    // 数据项名称
    mySunburst.bindingName = ['name', 'name', 'symbol'];
    // 在分层数据中生成子项的属性的名称。
    mySunburst.childItemsPath = ['groups', 'items'];
    mySunburst.endUpdate();
};
DataLoader.generateCollectionView(setSunburst);
 ```

### 2.2 python

```python
import plotly.express as px
import pandas as pd

A = ["A", "B", "C", "D", None, "E",
     "F", "G", "H", None]

B = ["A1", "A1", "B1", "B1", "N",
     "A1", "A1", "B1", "B1", "N"]
C = ["N", "N", "N", "N", "N",
     "S", "S", "S", "S", "S"]
D = [1, 13, 21, 14, 1, 12, 25, 1, 14, 1]

df = pd.DataFrame(
    dict(A=A, B=B, C=C, D=D)
)

fig = px.sunburst(df, path=['C', 'B', 'A'], values='D')
fig.show()

print(df)
```



## 修改记录

- 2021-09-05, 079组杨秀毓,陈梓钊,李梓芸补充内容
- 2021-08-30，张嘉乐更新模板
- 2021-08-10，郑鸿晓更新模板
- 2021-08-04，张嘉乐建立模板

[^1]: Collins C，Car pendale S，Penn G．Docuburst：Visualizing Document Content Using Language Structure ［J］．Computer

Graphics Forum，2009，28（3）：1039-1046．

[^2]: Muller G A．Wordnet：a Lexical Database for English ［J］．Communications of the ACM，1995，38（11）：39-41．
[^3]: 李燕,李新全,基于DocuBurst的动车组非结构故障数据可视化,第十一届中国智能交通年会大会论文集.567-572

