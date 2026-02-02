

# Motivating example：curve fitting
目前为止，acton value和state value都是以表格（数组）的形式来表示的，但是当空间很大或者空间连续时，存储和泛化能力都出现了问题。

有以下例子，例如有n个状态s，分别对应了n个state value，如果使用表格来存储，则空间为O(n)。但是如果找一条曲线去拟合这个过程，最简单的使用一条直线，则只需要存储直线的两个参数，从而极大的节省了内存。当然，这个近似的过程降低了数值的精度。而为了提高精度，可以提高拟合曲线的阶数。

但是value function approximation有一个重要的优势，即使有的状态还没被访问到，但是改变曲线的参数可能使其他状态的价值也更接近真实值。


# Algorithm for state value estimation
## objective function
![[620d095b-1e5a-4bed-945e-ce765a5f1b8a.png]]
首先让v(s)表示真实的state value，而v hat则是估计值，目标便是尽可能让估计值去接近真值，而当这个拟合函数的结构（曲线/神经网络）确定时，要做的就是调节这个w。这个过程可以看作一个优化问题，有如下的目标函数。
![[5fcc663f-4553-460c-9068-88fbc06c3ba5.png]]
要找到最优的w让目标函数最小。此处S是一个随机变量，因而有一个概率分布

一种假设是S是平均分布的，即所有的状态都同样重要；但实际并非如此，我们希望一些状态更重要，有更大的权重，从而有更小的估计误差。
![[4be1b48e-000c-4441-9245-ca5a9981ac41.png]]

从而引入第二种分布，stationary distribution稳态分布（马尔可夫过程中到达稳态后的分布）
![[9e99a426-d282-4dac-b8d3-275f707126f3.png]]

## optimization algorithm
有了上面的目标函数，接下来需要有优化算法，最直接的是梯度下降
![[e94d9014-6ace-4867-9a49-72b19b7784d2.png]]
然后用随机采样来替代期望
![[fe129eec-f0c7-4816-b644-e7d1d6168d82.png]]

但是注意到算法中有一个v pai st，这个是未知的，该如何去得到？使用前面的MC与TD去拟合。

# Sarsa with function approximation


# Q-learning with function approximation




# Deep Q-learning
