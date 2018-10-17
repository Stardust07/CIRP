# 问题描述

## 概述

给每架航班分配一个停机位, 要求在满足航班与停机位的兼容性以及同一停机位上最短过站时间间隔的情况下, 使停靠在近机位 (靠桥) 的航班尽可能多.

每个周期

要求每个客户不断供的情况下，配送方案，使得库存维护开销和路由开销的总和最小.


## 已知

- 车辆
  - 容量
  - 最短过站时间间隔
- 仓库
  - 单位开销
  - 容量
  - 初始库存
  - 每周期补充库存
- 客户
  - 单位开销
  - 容量
  - 初始库存
  - 每周期消耗库存

### 集合

| Symbol | Description  | Remark              |
| ------ | ------------ | ------------------- |
| $V$    | vehicle set  |                     |
| $T$    | period set   | ${0,1,...}$         |
| $N$    | node set     | $0$ is the supplier |
| $N^*$  | customer set | $N^*=N-\{0\}$       |

### 常量

| Symbol     | Description                                              |
| ---------- | -------------------------------------------------------- |
| $H_i$      | unit inventory holding cost for node $i$                 |
| $C_{ij}$   | routing cost for edge $(i, j)$                           |
| $Q^C_i$    | inventory holding capacity for customer $i$              |
| $Q^V_v$    | capacity of vehicle $v$                                  |
| $U^C_{ti}$ | cumulative consumption until period $t$ for customer $i$ |
| $U^S_t$    | cumulative supplement until period $t$ for the supplier  |
| $I^C_i$    | initial quantity for customer $i$                        |
| $I^S$      | initial quantity for the supplier                        |

## 约束

- 航班必须停靠在与之兼容的停机位上
- 同一停机位上停靠的任意两架航班过站时间间隔必须大于给定值


## 目标

停靠在近机位的航班数.

## 决策

每个客户送多少货



# 数据接口

## 输入数据格式

### 基本情况

见 `Protocol/GateAssignment.proto`.

### 数据约定

- 停机位数不超过 100
- 近机位数不超过 30
- 航班数不超过 400
- 每架航班禁止停靠的停机位不超过 8 个, 且一定有可以停靠的近机位


## 输出数据格式

### 基本情况

见 `Protocol/GateAssignment.proto`.
