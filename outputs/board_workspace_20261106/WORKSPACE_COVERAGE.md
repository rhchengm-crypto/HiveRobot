# A1–D8 离线路径覆盖

**已被 v3 场景修正取代：这是 v2 从 Home 起点的名义扫描。随后实机反馈在直接接近时碰桌；本表不证明完整桌面避碰。v3 强制 Clearance 并加入全局桌面高度约束，需重新扫描。**

已检查 40/40；完整路径通过 26；未实机执行。

P：声明的模型包络下路径通过；U：搜索预算耗尽；X：其他规划/复核失败；?：未完成。

**P 不表示实物抓取已验证；U/X 不证明物理不可达。**

| 排 | A | B | C | D |
|---|---|---|---|---|
| 8 | U | U | U | U |
| 7 | U | U | U | U |
| 6 | P | P | P | P |
| 5 | P | P | P | P |
| 4 | P | P | P | P |
| 3 | P | P | P | P |
| 2 | P | P | P | P |
| 1 | P | P | U | U |

## 格间及边界点

| 点 | X mm | Y mm | 结果 |
|---|---:|---:|---|
| between-0 | 55.0 | 55.0 | P |
| between-1 | 110.0 | 110.0 | P |
| between-2 | 165.0 | 165.0 | P |
| between-3 | 55.0 | 330.0 | U |
| between-4 | 110.0 | 385.0 | U |
| between-5 | 165.0 | 275.0 | P |
| between-6 | 5.0 | 5.0 | U |
| between-7 | 215.0 | 435.0 | U |

## 未通过点

- a7：planning_budget_exhausted
- a8：planning_budget_exhausted
- b7：planning_budget_exhausted
- b8：planning_budget_exhausted
- between-3：planning_budget_exhausted
- between-4：planning_budget_exhausted
- between-6：planning_budget_exhausted
- between-7：planning_budget_exhausted
- c1：planning_budget_exhausted
- c7：planning_budget_exhausted
- c8：planning_budget_exhausted
- d1：planning_budget_exhausted
- d7：planning_budget_exhausted
- d8：planning_budget_exhausted

每点的模型哈希、代码哈希、余量、失败详情及原报告路径保存在同名 JSON 总表中。
声明包络不含完整手指、线束、支架、其他棋子及全部自碰撞；真实机械限位尚未确认。
