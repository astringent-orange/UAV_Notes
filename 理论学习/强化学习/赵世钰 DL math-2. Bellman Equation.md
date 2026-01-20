
# 1. Motivating examples
为什么return重要
可以说明策略的好坏

如何计算return
一种是根据定义来计算
第二种是根据递推来计算，即一个状态的value依赖于其他状态的value

# 2. State Value
考虑这样一个trajectory
![[e8c4393b-7cd3-4cf7-9a05-0eea7bcc5687.png]]
有discounted return
![[356b7a27-e516-40f9-9b92-54a0e82593b6.png]]
于是有state value，即在一个确定状态下Gt的期望值，注意其实v也是policy（用pai表示）的函数，但是写在下标
![[bf3f2025-e4cb-4a4d-ba6c-5883f1c9c4ec.png]]

**return与state value的区别**
return是对于一条 trajectory所求的，而state value是对于多条trajectory求的平均


# 3. Bellman equation: Derivation
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


# 4. Bellman equation: Matrix-vector form
上面提到了贝尔曼方程对于所有s都是成立的，于是将所有公式写在一起然后进行整理，便可以得到矩阵-向量的形式。首先对于上面的公式进行演化
![[5ce4b182-adb1-4b02-ab51-8962e86e5b42.png]]
![[4dd02fd4-71ac-45aa-8d39-c042f8074863.png]]
相当于把表达式又写回去

然后要写为矩阵-向量形式
![[d184408c-592d-4256-b768-bcefffc3fccf.png]]
首先为所有的s赋予编号，然后按照编号排列公式并将转移矩阵展开，示例如下
![[dce5c957-baf8-4a37-bf07-2c4f916cddad.png]]

# 5. Bellman equation: Solve state value
给定一个policy，然后找到对应的state value的过程叫做policy evaluation。根据上面的贝尔曼的矩阵向量形式，有两种求解方式。第一种是closed-form
![[16882c97-0f77-4608-96b9-e58645249d5f.png]]
虽然形式很好看，但是在实际中，可能并不会使用。因为当状态空间过大时，求逆矩阵将会非常复杂。于是会用第二种，迭代的公式
![[2b5e6a5a-a906-4037-ad59-5b801a3c3ece.png]]
其思想在于，贝尔曼方程规定了真实的v pai必须满足这个等式
![[ff010f5b-7d16-461f-8b1d-567bd4dcfe20.png]]
而目前不知道真实的v pai，就随意猜测一个v0，然后带入右边等式，然后将算出来的结果设为v1，然后不停地重复这个过程，当k趋于∞时，vk趋于v pai。
为什么这个方法可行呢？利用高数中数列收敛的原理。当k->∞时，误差趋于0
![[dbc852ca-df33-41dc-8281-bd6b444b368f.png]]

# 6. Action value
state value是agent从一个状态出发，可以得到的avg return；action value是 agent从一个状态出发且采取某个动作，可以得到的avg return。其定义如下
![[21991911-906c-4fdb-b358-8c58bc6af863.png]]
可见该函数依赖于a与s。对比state value只依赖s。当然，两者都依赖策略pai。两者的联系如图，state value的公式中有一部分就是action value，相当于policy采取不同action的action value的加权和
![[4fcab620-9df3-47f3-8cef-c1f36e879550.png]]

联系贝尔曼方程，发现等式右边，括号中的部分就是action value的表达式
![[d72182ef-ccf9-403c-bac0-19fcd88a0997.png]]
![[9756641a-dae1-4a5f-94ec-70d323173f06.png]]
于是可以知道，如果有所有的action value，求个平均就有state value；如果知道所有的state value 也可以反推出action value