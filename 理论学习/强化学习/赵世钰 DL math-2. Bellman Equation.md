
# Motivating examples
为什么return重要
可以说明策略的好坏

如何计算return
一种是根据定义来计算
第二种是根据递推来计算，即一个状态的value依赖于其他状态的value

# State Value
考虑这样一个trajectory
![[e8c4393b-7cd3-4cf7-9a05-0eea7bcc5687.png]]
有discounted return
![[356b7a27-e516-40f9-9b92-54a0e82593b6.png]]
于是有state value，即在一个确定状态下Gt的期望值，注意其实v也是policy（用pai表示）的函数，但是写在下标
![[bf3f2025-e4cb-4a4d-ba6c-5883f1c9c4ec.png]]

**return与state value的区别**
return是对于一条 trajectory所求的，而state value是对于多条trajectory求的平均


# Bellman equation: Derivation
贝尔曼公式，描述了不同状态的state value之间的关系

首先Gt可以被写为以下两部分，immediate reward和future reward
![[1d71c439-80fa-4e99-b1a4-5db4090ac340.png]]
于是在对于期望的计算中可以将Gt拆开，有如下形式，然后分开计算两部分
![[7ef7e3d0-a7f6-49e5-8ea2-735ee41ede75.png]]

先对当前时刻reward的期望进行计算
![[5d6cdd69-7d76-4220-ac32-569d919df5c1.png]]
首先是根据全概率公式有，当前时刻期望=在当前状态下选择不同行为得到的期望；而对于某状态下，做出某动作得到的期望=可能的reward的期望
（即在某状态下做出什么动作不是确定的，得到多少reward也不是确定的）

然后对于未来的reward的期望进行计算
![[a81e0ac7-f2ff-4b8a-a157-7ff0a546725c.png]]
首先是从当前状态出发，可能跳转到不同的状态；而其中当前状态是可以去掉的（马尔可夫性）；而从下一个状态出发，能得到的return均值，就是s‘的state value；而从s到s’的概率，可以拆分为经过某个动作到s‘。

于是综上，有以下表达式，即贝尔曼公式
![[e58a05d6-15c4-4ef3-bc65-f414fb4799c8.png]]
对于空间中每一个状态s都有该公式成立。其中v(s)与v(s')是想要计算的state value。然后公式中有许多的概率，首先pai(a|s)就是policy；其次另外两个概率是environment model，这里先假设知道这个model。


# Bellman equation: Matrix-vector form
上面提到了贝尔曼方程对于所有s都是成立的，于是将所有公式写在一起然后进行整理，便可以得到矩阵-向量的形式。首先对于上面的公式进行演化
![[5ce4b182-adb1-4b02-ab51-8962e86e5b42.png]]
![[4dd02fd4-71ac-45aa-8d39-c042f8074863.png]]
相当于把表达式又写回去

然后写为矩阵-向量形式

