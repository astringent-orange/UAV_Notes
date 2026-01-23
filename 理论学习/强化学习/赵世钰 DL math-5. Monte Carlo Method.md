
从model-based到model-free。将VI与PI统称为model-based RL，但更准确地说可以称为dynamic programming。这里后续都是使用PI的方法，即每一步都找出当前策略下所有动作的价值。

# Motivating 
![[407f906b-e00d-4c1c-bae9-e463f6e33934 1.png]]
在之前的求解贝尔曼方程的过程中，不管是PI还是VI，我们都是已知环境模型，即以grid world而言是知道地图的样子，知道采取什么行为后会变更到什么状态，且了解做出做出行为后会得到多少reward；以数学而言，就是知道以下公式中两个p。
![[66818376-d40e-4844-b7eb-7a2d4906333b.png]]
但实际上，大多数时候是没有这样一张地图的。那么这个时候该怎么办？

一种最简单的方法，就是通过大量真实数据进行采样，求其统计平均

# Simplest MC-based algorithm
**Algorithm: MC basic** 如何将PI转换为一个model-free的算法

可以发现，关键在于如何求q
方法一，根据以下的迭代公式，已知v去求q
![[a16c43e9-860e-4fbe-8b66-a2126e61dcce.png]]
方法二，一个model-free的方法，使用q最初的定义
![[313ad9ba-0f0d-449f-a07a-1e935ace9bf2.png]]
这里通过许多个从状态s出发，选择动作a所获得的return的平均来估计该期望
![[85b422a5-a2be-4a50-a879-be64de13057a.png]]
也就是通过大量的数据来估计。总结也就是没有模型就要有数据，没有数据就要有模型，这些数据在RL中称为experience

于是有该算法的描述：
从一个初始策略出发，在任意第k个迭代中：
1. policy evaluation：获取所有的q(s,a)；这里是通过实际数据来求平均
2. policy improvement：让策略选择最好动作

细节：首先在上面MC过程中，为了获得一个状态s下某个动作a产生的影响，需要走完该路径上所有的状态。当然走完所有状态通常是不现实的，所以设置一个episode值控制走几步，表示往后看几步

# Use data more efficient
**Algorithm: MC exploring starts**
MC basic在实际中并不实用，因为效率很低；因此提出了以下两个改进方法
1. 更高效的数据利用
![[f5ca8c88-2a87-477d-9404-c2d701ffc27e.png]]
initial-visit：在MC basic中，从s1做动作a2探索一条路径结果只会更新s1，a2的价值，但是实际上还经探索过了s2，a4等情况，所以其实可以顺路更新这些情况的价值。其中，将经过一个state action pair称为对其进行了visit。

为了改进上述方法，又有以下两种情况。举例而言如图中，经过了两次s1 a2
every-visit：每一次经过s1 a2都进行一次估计
first-visit：第一次经过s1，a2才进行估计

2. 更频繁的策略更新
在MC basic中，计算一个动作的价值，需要实际探索许多次来求平均值，最后才能更新一次策略，这样太慢了。于是可以每探索一次，就将该次的return认为是action的价值，然后更新一次策略。

这样虽然抖动更大，但是仍然能够收敛


这些算法都可以称为General policy iteration GPI

综上，将两种方法结合起来，有了MC  exploring starts
![[9012aa2a-57ee-47a3-ad6a-fa6cb9d3f00b.png]]
这里有一个条件，选择的起始pair要能尽量访问所有pair；但是实际上这样无法保证所有sa对都能被访问，所以还是要遍历每一个sa对（first visiti）
此外，每次得到一条episode，都是倒推着从终点去访问并计算return


# MC without exploring starts
**algorithm: ε-greedy policy**
如何去掉exploring starts，引入soft policy：对每一个action都有可能去做选择。即将策略更改为随机性的（之前的策略是确定性的，只选择q最大的动作），这样当一个episode很长时，可以确保能访问到所有的sa对

如何实现soft policy呢？这里使用ε-greedy policy
![[12dd75b6-c10e-494b-ae9c-aa219421c36e.png]]
其中ε是0到1的正数，而|A(s)|是action的个数。从而以较大概率选择greedy action，但是其他action也能选择到。其中greedy action的概率比其他任何动作的概率都大
![[1d7a5d5e-9f7a-4324-af86-323c23a1f66b.png]]
为什么使用ε-greedy？平衡exploitation和exploration，即又要充分利用已知的高价值行为，又要去探索其他情况。当ε=0，那就变为greedy算法，只会选择已知的最佳行为；当ε=1，则所有行为有同样的概率。

如何与MC算法结合？
之前两个MC算法的policy improvement部分如下，greedy选择最佳的行为
![[1449c46b-e779-4097-90b4-34794b2df24a.png]]

现在改进后如下
![[5ee346d3-4019-4f9b-bca9-0c12715762d7.png]]
即策略的选择变化了

![[2c285674-b0de-4d97-b069-fba8749cab0c.png]]
伪代码算法如图，不同之处有从first visit变为every visit；因为现在改变后episode可能会很长，中间可能多次访问同一个sa，first visit可能会浪费数据