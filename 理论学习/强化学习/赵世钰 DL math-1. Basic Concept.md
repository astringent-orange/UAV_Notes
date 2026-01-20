
grid-world example：机器人在网格中进行移动

state：agent相对于env的状态
state space：所有state的集合
action：对于每一个状态可以采取的行为，a其实是s的函数，每一个状态可采取的行为是不同的
state transition：使用概率来描述，state transition probability，在不同状态采取了不同行为有不同概率切换到某状态
policy：告诉agent在不同state采取怎样的action，同样使用概率来描述

reward：当agent采取了一个action后得到的标量，用正负表示鼓励与惩罚；同样可以用条件概率来表示
trajectory：一个state，action，reward的链
return：一条traj上所有reward的总和
discounted return：加入衰减折扣
episode（trial）：对于有些任务长时间不会结束，称为continuing task

Markov Decision Process
集合：状态S，actoin A(s)，reward R(s, a)
概率分布：状态转移p(s' | s, a)，奖励p(r | s, a)
policy：采取动作的概率 pai(a | s)
马尔可夫性