
正式介绍强化学习的model-based算法。先总结一下，前面两章贝尔曼方程与贝尔曼最优方程，是在描述策略的好坏与如何找到最好的策略；而本章是介绍如何去求解两个方程（虽然前面两章已经有了部分介绍）。注意，目的一直是去找到最好的策略。

# Value iteration algorithm
在上一章的内容中，通过以下迭代方法求解了贝尔曼最优方程，该方法被称为 value iteration 值迭代
![[99369ef7-71fc-408d-9739-1dfd3d943bb6.png]]
具体而言，其分为两个步骤
1. policy update：对于给定的vk，求解出一个pai让右侧的内容最大
2. value update：将上面求得的最大值带入，求解vk+1


# Policy iteration
首先给定一个policy pai0，有以下两步
1. policy evaluation （PE）：前面提到过，给定一个策略，求解其贝尔曼公式的过程就是PE；一种是闭式的，一种是迭代的
![[bacc972d-840a-4bc7-98b4-ca18b94d6d05.png]]
2. policy improvement：在求出pai k之后根据以下公式求出一个更好的pai k+1的过程；实际上就是上一步中求出了v之后，有q，让pai直接选取q最大的动作即可
![[79ba872b-94d2-4768-b801-1d92ac1cc4f7.png]]
即如何找到最好的策略呢？PI首先要求出当前策略下每个状态的优略，从而求出每个action的优略；然后根据q去更新策略，即让策略选择最好的action


那么两种算法之间有什么关系呢？

