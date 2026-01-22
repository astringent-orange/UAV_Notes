
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
**Algorithm: MC basic starts**
MC basic在实际中并不实用，因为效率很低；因此提出了以下两个改进方法
1. 更高效的数据利用
![[f5ca8c88-2a87-477d-9404-c2d701ffc27e.png]]



2. 更频繁的策略更新



# MC without exploring starts
