
# Motivating examples
为什么return重要
可以说明策略的好坏

如何计算return
一种是根据定义来计算
第二种是根据递推来计算，即一个状态的value依赖于其他状态的value

# State Value
考虑这样一个trajectory
![[e8c4393b-7cd3-4cf7-9a05-0eea7bcc5687.png]]
有discounted return
![[356b7a27-e516-40f9-9b92-54a0e82593b6.png]]
于是有state value，即在一个确定状态下Gt的期望值，注意其实v也是policy（用pai表示）的函数，但是写在下标
![[bf3f2025-e4cb-4a4d-ba6c-5883f1c9c4ec.png]]

**return与state value的区别**
return是对于一条 trajectory所求的，而state value是对于多条trajectory求的平均


# Bellman equation: Derivation
贝尔曼公式，描述了不同状态的state value之间的关系

首先Gt可以被写为以下两部分，immediate reward和future reward
![[1d71c439-80fa-4e99-b1a4-5db4090ac340.png]]
于是在对于期望的计算中可以将Gt拆开，有如下形式，然后分开计算两部分
![[7ef7e3d0-a7f6-49e5-8ea2-735ee41ede75.png]]

先对当前时刻reward的期望进行计算
![[5d6cdd69-7d76-4220-ac32-569d919df5c1.png]]
首先是