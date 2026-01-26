
回顾之前的内容，在MC部分第一次介绍了model-free的方法，而TD是第二种，且前者是无增量的后者是增量式的

# Motivating Example
第一个例子，在已知一些采样xi的情况下求解EX。可以用RM方法，令g(w)=w-EX，从而有测量值w-x
同理，对于同样的情况，求解E(v(x))
最后，有w=E(R+γv(X))

# TD of state value
首先，TD即可以指一大类算法，也可以指一个特别的算法。
![[8aa976a0-56ef-47c1-b3bf-bd683cc73adf.png]]
首先TD需要一串experience，s和r交替，且这串数据是由给定的策略pai产生。TD会根据这一串数据来估计该策略的state value，算法如下
![[ae0b4e85-c98e-4993-9b29-16f9689cc466.png]]
注意第二个式子中的v(s)，这个没有下标的s表示state space中任何一个状态；而这个v是用来估计真实价值v pai的，下标t代表在t时刻的估计值

具体分析第一个式子，新的估计是由旧的估计加上一个修正项得到
![[c35148a5-5ea0-4153-82c7-90af9de73da5.png]]
为什么vt bar被称为TD target？因为想要vst朝着vt bar的方向改进
![[22126260-11a8-49f0-bbff-9dc7d62b1825.png]]
两边都减去一个vt bar，这样描述了两个时刻之间，vt与vt bar的差异；取绝对值，可以看到t+1时刻的差异更小了，所以说TD是将v向vt bar的方向推进
![[1c883f34-c77d-47b0-b53f-8fd751e2ec94.png]]


最后总结一下
TD只是在估计state value；不能估计action value；不能找出最优策略