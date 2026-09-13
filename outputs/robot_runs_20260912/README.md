# 2026-09-12 实机记录归档

本目录保存用户在当天提供的 12 份 Web 命令输出粘贴记录，以及锚点导入后“读取已保存数据”的 JSON。文件按开始时间命名；内容逐字取自对应附件，未重算关节数据。部分粘贴记录可能只是 Web 输出的截取，不能当作机器人端完整运行日志。

| 文件 | 附件 ID | 内容 |
|---|---|---|
| `134133_home_or_replay.log` | `424e5278-5cd8-435d-9cc6-4ea615b5d3ee` | Clearance 通过，放置 replay 在 final wrist 前被 arm_roll 残差挡下 |
| `134826_replay.log` | `76a64528-ed51-49c8-91b3-1301b1ca2e3d` | pre-wrist elbow、arm_roll 超差 |
| `135148_replay.log` | `b9d0f516-6f08-4878-a8eb-7599e7a209c1` | pre-wrist arm_roll 超差 |
| `135534_replay.log` | `45e1ea66-fbf2-4a99-b189-6bb8c65551df` | pre-wrist arm_roll 误差降到约 0.525°，仍未达到 0.5° |
| `140927_bishop_place.log` | `e18fdaf2-ac45-4954-ab4e-7c72e418b8d0` | white_bishop_place 单独回放验证通过 |
| `145303_full_placement.log` | `9960b295-34e8-45c9-b6eb-e8275608aedc` | White Bishop Placement 完整流程成功，Placement1 clearance 仍有 wrist_side 残差 |
| `153608_claw_release.log` | `725a790f-6b74-4b09-9af0-a70ea152dac9` | 新增 Claw Home 后的首轮回放 |
| `154430_claw_release_stable.log` | `3d2317b1-d3dd-40e8-ad9b-fbf5576917dc` | wrist 持续支撑下张爪，用户确认表现良好 |
| `205754_knight_c4.log` | `4c3a6aeb-437c-4443-8be3-d07abe6ed57a` | white_knight_c4 回放七关节验证通过 |
| `211251_placement_c4.log` | `05e7ff86-42df-4769-9b9b-51a331cd45db` | 夹爪压力检测给出 contact=false，流程被挡 |
| `211831_placement_c4.log` | `4d7e2de2-138e-461b-bb43-ecd62e79ab9b` | 再次 contact=false，用户目测已夹取 |
| `213421_placement_c4_success.log` | `442f85f5-defd-4bde-9583-994a766c6933` | 跳过空夹判断后完成夹取并到 Clearance；训练仍未达 0.5° |
| `geometry_status_after_import.json` | `8d3f8b6e-b20b-4109-8def-66cfc59c55c2` | 机器人端读取已保存数据：c1、c4、d4 三个锚点 |

权威姿态与确认元数据另见 `scripts/data/chess_crown_geometry_v2_8.json`。机器人端的 saved moves、Clearance 与局部学习 bias 文件不在本目录，下一次需从机器人单独备份，不能用仓库历史副本覆盖。
