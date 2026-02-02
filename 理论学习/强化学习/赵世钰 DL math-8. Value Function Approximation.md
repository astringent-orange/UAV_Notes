

# Motivating example：curve fitting
目前为止，acton value和state value都是以表格（数组）的形式来表示的，但是当空间很大或者空间连续时，存储和泛化能力都出现了问题。

有以下例子，例如有n个状态s，分别对应了n个state value，如果使用表格来存储，则空间为O(n)。但是如果找一条曲线去拟合这个过程，最简单的使用一条直线，则只需要存储直线的两个参数，从而极大的节省了内存。当然，这个近似的过程降低了数值的精度。而为了提高精度，可以提高拟合曲线的阶数。

但是value function approximation有一个重要的优势，即使有的状态还没被访问到，但是改变曲线的参数可能使其他状态的价值也更接近真实值。


# Algorithm for state value estimation
## objective function
![[620d095b-1e5a-4bed-945e-ce765a5f1b8a.png]]
首先让v(s)表示真实的state value，而v hat则是估计值，目标便是尽可能让估计值去接近真值，而当这个拟合函数的结构（曲线/神经网络）确定时，要做的就是调节这个w。

# Sarsa with function approximation


# Q-learning with function approximation




# Deep Q-learning
