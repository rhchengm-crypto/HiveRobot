"""Finalize the handoff only after actual public Release and Git verification."""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
read = lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
r = read(OUT/'publication_receipt.json')
m = read(OUT/'publication_manifest.json')
assert r['status'] == 'all-experimental-code-and-data-published-and-verified'
assert r['release_asset_count'] == 31 and r['payload_asset_count'] == 29
assert r['git_text_files_verified'] == 959 and r['local_inventory_files_verified'] == 1000
body = '''## 13. 今日GitHub发布索引与完整性回执（已完成）

本次先推送当前代码、模型、参考库及交接，再分五批发布全部文本实验记录，最后从本机上传原始数据附件。没有用Jetson直接访问GitHub，没有发起新运动、Home、使能/失能或Web重启。公开发布保留现场原始失败证据和版本历史。

- 仓库：[HiveRobot](https://github.com/rhchengm-crypto/HiveRobot)。
- 正式数据页：[experiments-2026-11-08](https://github.com/rhchengm-crypto/HiveRobot/releases/tag/experiments-2026-11-08)。
- 最先推送并核对的代码/初次交接提交：`CORE_COMMIT`，29个文件。
- 原始文本证据5批、959个文件、114,527,893字节，最后一批提交：`TEXT_COMMIT`。本节、索引与最终回执的文档提交在其后，不在文件内自引用自己的commit。
- 发布总索引：[outputs/github_sync_20261108/README.md](outputs/github_sync_20261108/README.md)。机器清单：[publication_manifest.json](outputs/github_sync_20261108/publication_manifest.json)。

### 13.1 代码、运动记录、图像三部分的对应

| 部分 | 已发布内容 | 如何对应每轮 |
|---|---|---|
| 代码、模型、配置、测试和验收登记 | `scripts/`当前源码、v33支持、完整帧求解修复、非兵范围过滤、13归档/12有效库；第9/11节及各轮冻结源码/模型保留精确SHA | 本交接时间线与逐轮模型表；库中原日志、登记和源模型SHA；不要把当前15a9求解模块倒称历史运行版本。 |
| 运动控制、诊断、拟合和同步记录 | 959个可在Git浏览的文本文件；19个原控制/准备/分析ZIP；另有收尾快照和缺项补包 | 14个`outputs/*20261108/`实验目录、原`manifest.json`、`execution_audit.json`、`trial_summary.json`、验收追加历史和拟合数据；[every_file_coverage.json](outputs/github_sync_20261108/every_file_coverage.json)逐文件对应Git或ZIP成员。 |
| RGB-D与辅助图像 | 7个原始无损RGB-D ZIP，合计3,690对、461,975,551压缩字节；4张独立辅助图像212,482字节ZIP | A7重复340对、A7 Home437对、A8初轮638对、其Home480对、v33轮702对、其Home419对、lower25轮674对；同前缀控制包/轮次目录对应。未去重删帧。 |

Release为**31个附件**：29个实验payload，共**571,848,294字节**（约571.85MB），加机器清单与恢复脚本2个元数据附件。29个payload由26个原ZIP、辅助图像包、收尾runtime包、26缺项补包组成。26原ZIP为520,613,431字节，压缩原件字节保持不变；不要把相互重叠的快照相加当成独立实验数。

### 13.2 完整性、远端补齐及最后Home

所有31附件已核对GitHub计算的SHA256、字节长度及`uploaded`状态，正式Release已解除draft。另以不登录的公开下载实际核对机器清单、恢复脚本及一个原ZIP的SHA，见[publication_receipt.json](outputs/github_sync_20261108/publication_receipt.json)和`release_remote_verification.json`。

959个Git文本文件已在已推送提交中逐个读取Git blob、核对原始字节和SHA，不只查看工作区文件存在。全部1,000个本地实验文件有发布去向；26个Jetson补充快照也逐成员核对。13个生成的Python字节码缓存不发布，原源码保留。138,752,252字节滚动motion JSONL超过Git单文件限额，通过原控制ZIP完整保留并由覆盖清单定位，没有丢弃；连续控制帧及其他文本仍可在Git浏览。

只读对照Jetson整个当日实验目录，7轮原始RGB-D均有本地完整原包；补齐26个未在本地的非原始帧记录，包含采集报告/原始RGB-D成员清单/同步回执/早期深度快照与Web日志。活动`web.log`按明确截止点保存为有限快照，保留早期清单前缀并标明未封存，不停服务器来追求“最终不再增长”。源目录、原图、原深度、失败记录均未删除。

`final_runtime_snapshot_20261108.zip`完整保存219个Jetson源码/模型/学习/全局日志/当日连续控制文件，原始252,617,956字节，压缩50,290,144字节，SHA256 `54f449b2bb65e600a31d70ad6ccedeb70b376cb5870345b0acda7bf1ed60979f`。ZIP CRC及每个成员SHA已在本机核对。它包括用户20:57:59已结束的最后Home/保持，快照前后Web最后动作未变。本次收尾没有运动。

收尾快照列出9处本机历史版本/滚动日志与现场不同：3个滚动日志、4个历史控制/Web源码、2个历史测试。两边原字节分别保留，不把旧本机副本覆盖现场，也不把不同版本混写成同一运行源。当前主控制、同串口支持、scope模块和两份运行模型与收尾现场相符；详见[final_snapshot_verification.json](outputs/github_sync_20261108/final_snapshot_verification.json)。

### 13.3 在Mac或其他本机恢复

```bash
# 只列清单，无下载/解压/运动。
python outputs/github_sync_20261108/restore_publication_data.py
# 下载7轮原始RGB-D并逐包核SHA。
python outputs/github_sync_20261108/restore_publication_data.py --group nov8-rgbd --download
# 下载全部29个payload并核对。
python outputs/github_sync_20261108/restore_publication_data.py --group all --download
```

每轮解压到独立目录，按其冻结源和模型恢复。新拟合候选仅为离线实验产物，恢复文件不等于启用模型。旧Mac聊天迁移包仍不公开；SSH私钥、凭据库和私有附件不在本次发布范围。继续工作以本交接和当前GitHub版本为准，不能用昨日快照覆盖最新学习数据。

今日59项相关离线测试通过；现场通过范围仍为相应空载接近、下降、保持及非兵姿态认可。发布完成不增加闭爪带载、精确TCP、机械硬限位或全棋盘已验证的结论。下一轮仍等用户指定，不自动运动。
'''
body = body.replace('CORE_COMMIT',m['core_commit']).replace('TEXT_COMMIT',m['text_evidence_commit'])
p = ROOT/'HANDOFF_20261108.md'
original = p.read_text(encoding='utf-8')
assert '## 13. 今日GitHub发布索引与完整性回执' in original
p.write_text(original.split('## 13. 今日GitHub发布索引与完整性回执')[0]+body,encoding='utf-8')
print(json.dumps(dict(handoff_updated=True,release_assets=r['release_asset_count'],payload_bytes=r['payload_bytes'],motion_issued=False)))
