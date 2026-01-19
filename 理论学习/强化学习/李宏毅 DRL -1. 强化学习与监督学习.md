
不知道正确答案的情况，但是通常知道什么是好和不好

机器学习：找一个 函数，而在强化学习里如何表现？在RL里有一个actor与env，actor从env获得observation，然后输出action，而从ob到action的过程就是这个function。而env会对action给出reward，要找一个fun最大化reward的总和，但是reward并不一定是即时的

RL与ML，同样第一步是带有参数的fun：输入ob，经过网络输出采取的act。对于采取的动作，将得到的分数作为概率，而不是直接选取分数最高的。

第二步，定义这个“Loss”。整场游戏（episode）的过程中，有一个total reward

第三步，关于优化。将s与a的交叉序列称为trajectory，而reward是s，a的函数，找出actor的参数以最大化R。注意，a是具有随机性的。且env的运作可能并不是已知的，只是黑盒子。此外，reward函数并不是可学习的，且可能具有随机性。