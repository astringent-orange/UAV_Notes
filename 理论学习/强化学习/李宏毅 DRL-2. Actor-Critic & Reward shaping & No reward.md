
critic：对于一个actor，当其看到s或者做出a时，对其进行评价

一个critic举例：
value function：估测某个actor在看到s后会得到的discounted cumulated reward。（正常来说是在完成整个episode之后得到，这里的意义是提前预测）

如何训练critic
MC 蒙特卡洛方法：观察一个actor θ和环境完整互动多次，就有对应的{s,a}和G，用这些收集的资料去训练模型

temporal-difference TD 时序差分方法：有的game持续时间很长，甚至不会结束，用MC并不合适。因为V(st)=γV(st+1)+rt，即两个时刻间的评价值之间有相对关系，两者做差可以得到rt。于是只需要st，st+1与rt也可以训练模型。

使用MC和TD可能得到的结果是不一样的


policy
Ver 3.5 将b设置为V，即评价A=G-V。因为V是看到某个状况后的期望评价，如果执行结果大于平均结果，认为其是好的

Ver4 在上面的公式中，V是一个平均值，而G是一轮的值，看起来不太准。令A=rt+V(st+1)-V(st)。其中rt+V(st+1)即当前奖励加上之后的期望。advantaged actor-critic，A2C


具体而言
actor-critic网络，其输入的内容都是相同的，但是输出部分不同，于是可以共享一部分网络结构


reward shaping
目前通过actor与env互动得到reward，对于reward整理得到分数A。如果reward几乎都是0，只有少数巨大的reward（sparse reward），例如下围棋只在最终结束时会有reward，甚至如让机械臂打螺丝，在随机时几乎不会有reward。于是想办法提供一些额外的reward，来帮助学习（reward shaping）。

例如不仅对于一些结果给reward，还可以对一些行为进行reward。可见reward shaping是需要domain knowledge的

curiosity - reward shaping：如果机器看到了新东西（有意义）则进行加分


No reward
去定义reward有时都是困难的，一般只在游戏环境中比较好定义。

imitation learning
虽然仍然有actor与env进行互动，没有reward，但是有expert的环境互动的示范。和监督学习之间有什么区别？监督学习是让机器去“behavior cloning“，这样一是机器不会做出示范以外的行为，但是通常在RL中机器会面对的情况是无法被完全囊括的；二是机器可能会学到一些不相干的行为

inverse RL
之前的过程，是从reward经过RL去优化actor；现在反过来，expert经过inverse RL去反推reward。其最基本的概念：老师的行为是最好的（并不是完全照搬老师行为）
具体过程如下：
初始化一个actor，在每一轮训练中，actor会自己与环境互动收集到一些traj，现在要定义一个reward，让老师得到分数比actor得到的reward更高，从而更新reward fun，然后根据这个reward fun去更新actor。这个结构和GAN很相似。

这种方法常常用于训练机械臂