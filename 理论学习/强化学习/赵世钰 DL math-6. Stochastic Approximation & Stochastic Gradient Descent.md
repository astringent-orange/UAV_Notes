
# Motivating
![[48d7774e-7e00-4057-952a-fbcfd9e2e483.png]]
回顾之前的内容 mean estimation，通过统计平均值去近似X的期望

如果求这个“统计平均值”？有两种方法
1. 通过真实采样，等所有数据收集完成，然后加在一起求平均，耗时长
2. 增量式方法，来几个计算几个
![[b8ce2642-5502-44b4-86ef-b6bf9dc54538.png]]
通过迭代的方法，每得到一个采样更新一次平均

![[7a9a27fd-561c-47e2-b345-896565cd7448.png]]
如果将迭代算法改成这样，是否还能收敛到期望值？当αk满足一定条件时是可以的。而这种算法也是SA和SGD的一种特殊形式。

# Robbins-monro algorithm



# Stochastic gradient descent



# BGD MBGD SGD