
回顾之前的内容，在MC部分第一次介绍了model-free的方法，而TD是第二种，且前者是无增量的后者是增量式的。简单地说，MC即使再快，也至少要“完成”一次实验后才能求得价值期望；而TD是在实验的过程中即可动态更新，这样减少了时间的浪费，加快了收敛的速度。

# Motivating Example
第一个例子，在已知一些采样xi的情况下求解EX。可以用RM方法，令g(w)=w-EX，从而有测量值w-x
同理，对于同样的情况，求解E(v(x))
最后，有w=E(R+γv(X))

# TD of state value
TD即可以指一大类算法，也可以指以下这个特指的算法。
![[8aa976a0-56ef-47c1-b3bf-bd683cc73adf.png]]
TD需要一串experience，由s和r交替构成（或者是三元组的集合），且这串数据是由给定的策略pai产生。TD会根据这一串数据来估计该策略的state value，算法如图。要注意区分不同的符号：首先要求的是v pai (s)，代表当前策略下任意状态的价值，而现在vt是在不断迭代以接近v pai，其中t代表第t个时刻的估计值；st代表某一个状态，st+1是下一个时刻的状态，vt(st+1)并不是未知量，第二个式子中的s代表任意一个状态。

所以说可以理解为每次根据一个三元组，更新一个状态的价值，而其他状态的价值不变。

具体分析第一个式子，新的估计是由旧的估计加上一个修正项得到
![[c35148a5-5ea0-4153-82c7-90af9de73da5.png]]
为什么vt bar被称为TD target？因为想要vst朝着vt bar的方向改进
![[22126260-11a8-49f0-bbff-9dc7d62b1825.png]]
两边都减去一个vt bar，这样描述了两个时刻之间，vt与vt bar的差异；取绝对值，可以看到t+1时刻的差异更小了，所以说TD是将v向vt bar的方向推进
![[1c883f34-c77d-47b0-b53f-8fd751e2ec94.png]]
那么vt bar具体是什么？回忆之前的贝尔曼方程，这里的下标稍微有些不同，即rt+1代表的是状态st的立即回报（reward）；而γ是衰减因子。

最后总结一下，TD只是在估计state value；不能估计action value；不能找出最优策略。

数学上说，TD就是在没有模型的情况下求解贝尔曼公式
推导过程如下，用RM去求解贝尔曼公式，于是有以下这个函数，想要求g=0
![[1c5b07f7-de27-46ea-9957-32989833cafe.png]]
而g的带噪观测如下
![[c68effc3-bd0d-4a85-ba6f-abc2a8830e2c.png]]
从而有RM的公式如下，已经和TD十分接近
![[b3c7a879-0f24-4af1-b609-b346933732d6.png]]
其中右侧有一个v pai，需要替换成vt

然后将TD与MC进行比较
![[272f1bb1-042a-41a8-996b-fd1a369ae2b0.png]]

# TD of action value: Sarsa
在前面的算法中，使用TD估计了state value，但是在PI中还是要通过计算action value然后选择那个最好的action，因而还是需要额外的步骤。而sarsa则可以直接估计action value。

同样，先直接给出算法的公式如下。给定一个策略，然后有一串经验，由状态、动作、奖励构成（比之前多一个奖励），或者可以表示为五元组(st,at,rt+1,st+1,at+1)的集合，而sarsa也是这五个元素的首字母缩写
![[4f934854-4456-499e-beac-d24845f54eab.png]]
可以看到公式和之前的TD形式相近，只是将v修改为q
![[15c19043-0ddf-4385-8cac-cea3ad57c85f.png]]

而将上面的内容与一个policy improvement结合起来，成为一个完整的过程，也称为sarsa。这里选择使用ε-greedy来选择策略。

下面介绍两种Sarsa的变形

## Expected Sarsa
![[f98d57af-8768-4124-86eb-d0748ab65b90.png]]
相当于将sarsa中右侧的qt改变为了vt，然后不用使用at+1；但是计算量增大了
![[a1fef474-eaec-4498-99c3-256ef1f03374.png]]


## n-step Sarsa
可以将sarsa与MC统一在一起
![[05e757c0-ed52-414d-8eca-1618f1ec7e80.png]]观察action value是怎么求的，可以根据如何对Gt进行分解有不同的理解。sarsa将Gt分解为一个真实reward与对之后的价值估计，而MC则将Gt全部分解为真实的reward。而n-step则是选取了n步真实reward。

而再对数据进行对比，sarsa只需要一个5元组就可以进行计算，而MC则需要所有5元组进行计算（等整个实验结束），而n-step需要n个5元组（初始等n次）进行计算

# TD of optimal action value: Q-learning
回顾sarsa，是直接估计action value，然后需要与policy improvement结合才能实现一个完整迭代过程。而Q-learning直接估计最佳的action value，而不需要在两个操作之间切换。同样，先给出算法如下
![[48ff6f47-0c36-4751-b438-40bfc81aff15.png]]
从结构上说和sarsa是相同的，只是target不同。而Q-learning所求解的数学问题如下，是在求解一个贝尔曼最优方程
![[4e2a0979-adc4-4707-b2f9-6589a3f6c504.png]]

接下来介绍Q-learning的一些性质，首先要正式引入两种概念：on/off-policy
在TD中有两种策略：behavior policy用来与环境交互得到experience的策略，target policy用来改进的策略；当behavior policy与target policy相同时，则称为on-policy，反正则称为off-policy

显然off-policy的好处在于可以用别人的经验去优化策略：例如behavior policy的探索性很强，而target policy是greedy的