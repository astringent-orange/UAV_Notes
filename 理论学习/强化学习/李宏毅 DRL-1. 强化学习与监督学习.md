
不知道正确答案的情况，但是通常知道什么是好和不好

机器学习：找一个 函数，而在强化学习里如何表现？在RL里有一个actor与env，actor从env获得observation，然后输出action，而从ob到action的过程就是这个function。而env会对action给出reward，要找一个fun最大化reward的总和，但是reward并不一定是即时的

RL与ML，同样第一步是带有参数的fun：输入ob，经过网络输出采取的act。对于采取的动作，将得到的分数作为概率，而不是直接选取分数最高的。

第二步，定义这个“Loss”。整场游戏（episode）的过程中，有一个total reward

第三步，关于优化。将s与a的交叉序列称为trajectory，而reward是s，a的函数，找出actor的参数以最大化R。注意，a是具有随机性的。且env的运作可能并不是已知的，只是黑盒子。此外，reward函数并不是可学习的，且可能具有随机性也是一个黑盒子。

如何控制actor。当做出一个act后，会得到一个reward，通过reward来判断是否应该做该act。更准确来说，使用reward来得到对于该act的评价

如何得到act的评价
Ver0。直接令评价=一步的reward，这样会造成模型短视，因为act会有后续影响，其次reward是可能有延迟的。

Ver1。将act之后所有reward求和为G（cummulated reward），将G作为评价。但是如果某一个过程非常长，当前动作对于将来的结果影响会变小，故这样的评价也并不完善。

Ver2。在上述求和过程中，对于每一个reward乘以一个衰减因子γ^i-1 。于是又有一个G（discounted cummulated reward）。

Ver3。好或者坏是相对的，例如如果某个过程中得到的reward都是正的，但是有大有小，于是需要做一个标准化。方法1，在所有的G上减去一个b，baseline，让G有正有负。


policy gradient
首先随机初始化模型的所有参数。用actor去与环境互动，得到一长串的{si,ai}，于是计算对应的Ai，然后求得最终的loss，用loss去更新一次model。

这里与监督模型不同的地方在于，收集资料的过程是在循环中的，收集一次资料只能更新一次model，导致RL的训练很慢。->收集资料的actor和训练的actor最好是同一个，这样称为on-policy。也有off-policy，要求模型能知道自己与收集资料的actor之间的不同。

exploration
前面提到，actor在选择行为时是具有随机性的，这样让模型可以尝试一些从没做过的事情。于是有的方法在训练时会扩大这种随机性。


