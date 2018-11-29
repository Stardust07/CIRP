# IRP-SDK使用注意事项

提交到checker的数据有如下约定

* routeInPeriod：当前周期t内所有车辆的路线集合，有对应函数添加。

* route：当前周期t内，某一车辆v的路线，有对应函数添加。

* delivery：记录当前周期t内，某一车辆v的路线上的一个节点的送货情况，有对应函数添加、以及设置数据成员的值。它有两个成员，见“\Protocol”下delivery的定义。①添加的顺序是：若路线上有客户节点，先按访问顺序逐个客户节点，最后再添加仓库节点，例如路线0$\to$3$\to$4$\to$2$\to$5$\to$1$\to$0，加入的顺序为342510；若路线上没有客户节点（亦即不送货），则仓库节点不需要加，但是得添加“route”，尽管该“route”是空的。②对于仓库而言，其set_quantity(X)中的“X”是当前周期车辆送货量的总和取负，是一个负数。

除此之外，应该在“\Deploy\Solution”文件夹下创建这样的文件夹：Instances_highcost_H3、Instances_highcost_H6、Instances_large_highcost、Instances_large_lowcost、Instances_lowcost_H3、Instances_lowcost_H6。

==gurobi的结果会有精度误差（用3.99999999999表示整数4），所以在用gurobi的变量值给整数类型变量赋值的时候，需要进行转换。==
