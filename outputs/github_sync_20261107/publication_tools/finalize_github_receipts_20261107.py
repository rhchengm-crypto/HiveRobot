from collections import Counter
from datetime import datetime,timezone
import hashlib,json,time,urllib.request
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'outputs/github_sync_20261107'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
save=lambda p,x:p.write_text(json.dumps(x,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
remote=read(out/'release_remote_verification.json');manifest=read(out/'publication_manifest.json')
assert remote['draft'] is False and len(remote['assets'])==59
expected={p.stem:read(p) for p in (out/'asset_receipts').glob('*.json')}
assert len(expected)==59
verified=[]
for asset in remote['assets']:
    name=asset['name'];r=expected[name]
    assert asset['size']==r['bytes'] and asset['state']=='uploaded' and asset['digest']=='sha256:'+r['sha256'],name
    assert '/experiments-2026-11-07/' in asset['browser_download_url']
    verified.append(dict(name=name,url=asset['browser_download_url'],bytes=asset['size'],sha256=r['sha256']))
public_probes=[]
for name in ['publication_manifest.json','restore_publication_data.py','a7_v32_execution_20261107__plan_staging.zip']:
    row=next(r for r in verified if r['name']==name)
    for attempt in range(3):
        try:
            request=urllib.request.Request(row['url'],headers={'User-Agent':'HiveRobot-public-verification'})
            with urllib.request.urlopen(request,timeout=60) as response:raw=response.read()
            assert len(raw)==row['bytes'] and hashlib.sha256(raw).hexdigest()==row['sha256']
            break
        except Exception:
            if attempt==2:raise
            time.sleep(2)
    public_probes.append(dict(name=name,unauthenticated_download_sha256_verified=True))
report=dict(status='all-experimental-assets-published-and-verified',checked_at_utc=datetime.now(timezone.utc).isoformat(),
    release_url=remote['url'],release_draft=False,release_asset_count=59,payload_asset_count=57,
    payload_bytes=manifest['asset_bytes'],all_asset_bytes=sum(r['bytes'] for r in verified),
    github_sha256_matches_all_assets=True,public_download_probes=public_probes,
    nov7_rgbd_sessions=12,nov6_backfill_sessions=15,nov7_rgb_depth_pairs=5100,nov6_backfill_rgb_depth_pairs=10360,
    text_evidence_files=961,core_commit=manifest['core_commit'],text_evidence_commit=manifest['text_evidence_commit'],
    assets=verified,private_chat_export_published=False,motion_issued=False,
    pending_research=['all-board physical coverage','loaded grasp and lift','physical verification of new fault recovery branch'])
save(out/'publication_receipt.json',report)
md=root/'HANDOFF_20261107.md'
s=md.read_text(encoding='utf-8').split('## 12. 今日 GitHub 发布与恢复索引')[0]
s+='''## 12. 今日 GitHub 发布与恢复索引（已完成）

本次先把最新代码、模型、参考库和交接推送GitHub，再分批发布原始控制证据，最后从本机上传无损图像/深度和运行快照。保留GitHub已有的其他项目提交，没有强制覆盖远端。归档过程中没有发起运动、Home、使能/失能或重启Web。

- 仓库：<https://github.com/rhchengm-crypto/HiveRobot>
- 正式数据页：<https://github.com/rhchengm-crypto/HiveRobot/releases/tag/experiments-2026-11-07>
- 最新代码/文档优先发布并核对的提交：`ff386d4b4b5b702402bd4453b7a74c0950643745`（包含本地源码快照`0a54ab5faa4c8161ee54b352bb9a3f67f2de99ff`并合并其他远端工作）。
- 原始文本控制证据6批全部发布的末提交：`ff5d166b575b1ad917597798e78d8f523e86d547`，961个文件、220,827,984原始字节。逐批远端确认见`outputs/github_sync_20261107/text_evidence_receipt.json`。
- 本节与最终发布回执的收尾提交在上述提交之后；可从GitHub文件历史查看，不在文件中自引用自身commit。

### 12.1 三部分对应

| 部分 | 内容和规模 | 位置与对应 |
|---|---|---|
| 代码、模型、标定与参考库 | 当前运行/开发源码、测试、JSON、批准STEP、历史每轮冻结版本、入库和验收登记 | `scripts/`、`source_geometry/approved_arm_reference.step`、本交接第4/9节；`core_manifest.json`记录初次发布工件SHA |
| 运动控制及研究记录 | 961个文本/连续控制/冻结源码与模型/诊断文件；另外28个原控制、准备和修复ZIP保持原字节 | 本仓库`outputs/*20261107/`，Release分组`nov7-control-and-preparation`；A7v29同时包含初始支持快照和`control_final.zip`，不能把未关闭trace的初始快照当作最终数据 |
| RGB-D与媒体 | 今日12组，5,100组RGB/深度；补齐昨日15组，10,360组RGB/深度；24个辅助图像/数组无损保留 | Release的`nov7-rgbd`、`nov6-rgbd-backfill`、`auxiliary-media`；逐项链接/字节/SHA见`publication_manifest.json`和`README.md` |

另有`final_runtime_snapshot_20261107.zip`，79个当前Jetson源码、模型、学习配置和完整运行日志，原始183,645,498字节。它补齐A7采集后又发生的Home与保持及全局motion/model日志，未只保留最后一次板面动作。

### 12.2 完整性与补齐范围

正式Release为**59个附件**：57个实验payload，合计1,867,040,745字节（约1.87GB），加`publication_manifest.json`和`restore_publication_data.py`两个下载元数据文件。所有59个附件的GitHub计算SHA256、长度及uploaded状态均与本机原件一致；另外无登录实际下载了清单、恢复脚本和一个原ZIP核验公开URL及SHA。回执为`outputs/github_sync_20261107/publication_receipt.json`，它区分代码、Git文本和二进制资产，不把生成文件等同已上传。

昨天handoff仅明确列出末两轮RGB-D欠缺；本次按Jetson整个11月6日实验目录和本地原包实际对照，发现共有15个未完整归档session，现已全部补齐。补齐原始数据6,999,873,988字节、20,750个原文件，无损压缩1,152,395,448字节；15组每个ZIP均核对远端/本机外包SHA、ZIP CRC和内部每个文件SHA，再从本机上传GitHub。旧“尚未回传”状态作为历史保留，新完成状态以`remote_rgbd/download_receipt.json`及`nov6_backfill_assets_receipt.json`为准。

没有删除近似帧或原始数据。所有Jetson源目录、本机原ZIP与历史失败证据保留。原来已在Git分片发布的两组11月6日RGB-D沿用旧恢复入口，不重复声称本次重新拍摄。

### 12.3 恢复、版本与隐私边界

```bash
# 在仓库根目录列出附件，暂不下载。
python outputs/github_sync_20261107/restore_publication_data.py
# 只取今日12组RGB-D并校验。
python outputs/github_sync_20261107/restore_publication_data.py --group nov7-rgbd --download
# 取全部57个实验payload并校验。
python outputs/github_sync_20261107/restore_publication_data.py --group all --download
```

每个ZIP解压到独立证据目录，避免不同轮次的同名源码互相覆盖。逐文件/逐轮模型版本仍按第4节及冻结manifest，不将当前代码冒充所有历史轮次。

收尾只读状态与Home原始日志在`outputs/github_sync_20261107/latest_home_evidence/`。Home `arm_run_1794113540677_56031.txt`，保持`arm_run_1794113621755_56254.txt`，最后Web记录服务器时间20:53:47完成；这是外部已发起动作的日志采集，本次发布没有运动。`final_snapshot_verification.json`另列8处本机历史副本/旧适配器/日志与现场字节差异，包括Clearance学习缓存；现场权威值原样保存在final runtime ZIP中，不以旧本机data整目录覆盖Jetson。

私人Mac迁移ZIP包含完整聊天和个人附件，继续保留本机供用户传Mac，**未公开上传GitHub**；账户登录文件和SSH私钥不纳入发布。`.gitignore`已明确排除该私人快照。Mac原ZIP早于本次最终交接和数据补齐，下次以GitHub本交接/发布索引补充。

相关138项离线回归通过，见`offline_verification.txt`和`verification_summary.json`。A7空载现场验收、未测试闭爪带载/新故障恢复分支、唯一物理根因未独立证明等研究边界没有因上传完成而改变。
'''
md.write_text(s,encoding='utf-8')
print(json.dumps({k:report[k] for k in ('status','release_asset_count','payload_bytes','github_sha256_matches_all_assets')}))
