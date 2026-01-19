
critic：对于一个actor，当其看到s或者做出a时，对其进行评价


一个critic举例：
value function：估测某个actor在看到s后会得到的discounted cumulated reward。（正常来说是在完成整个episode之后得到，这里的意义是提前预测）

如何训练critic
MC 蒙特卡洛方法：观察一个actor θ和环境完整互动多次，就有对应的{s,a}和G，用这些收集的资料去训练模型

temporal-difference TD 时序差分方法：有的game持续时间很长，甚至不会结束，用MC并不合适。因为V(st)=γV(st+1)+rt，即两个时刻间的评价值之间有