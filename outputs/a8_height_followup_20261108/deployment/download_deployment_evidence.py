from pathlib import Path
import base64,hashlib,io,json,subprocess,zipfile
HERE=Path(__file__).resolve().parent
receipt=json.loads((HERE/'deployment_receipt.json').read_text())
opts=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5']
code="""from pathlib import Path
import base64,hashlib,io,json,zipfile
root=Path('/home/nvidia/hive_robot/DM_Control_Python').resolve()
locations=LOCATIONS
buf=io.BytesIO();manifest={}
with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for label,value in locations.items():
  folder=Path(value).resolve();assert root in folder.parents
  for p in sorted(folder.rglob('*')):
   if not p.is_file():continue
   assert folder in p.resolve().parents
   name=label+'/'+p.relative_to(folder).as_posix();data=p.read_bytes()
   manifest[name]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()};z.writestr(name,data)
 z.writestr('manifest.json',json.dumps(manifest,indent=2))
data=buf.getvalue();print(json.dumps({'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'files':len(manifest),'data_base64':base64.b64encode(data).decode()}))
""".replace('LOCATIONS',repr({'stage':receipt['stage'],'backup':receipt['backup']}))
r=subprocess.run(['ssh']+opts+['nvidia@192.168.201.199','python3 -'],input=code.encode(),capture_output=True,timeout=45)
assert r.returncode==0,r.stderr.decode(errors='replace')
row=json.loads(r.stdout);data=base64.b64decode(row.pop('data_base64'))
assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256']
p=HERE/'deployment_evidence.zip';p.write_bytes(data)
with zipfile.ZipFile(p) as z:
 assert z.testzip() is None
 manifest=json.loads(z.read('manifest.json'))
 assert set(z.namelist())==set(manifest)|{'manifest.json'}
 for name,expected in manifest.items():
  content=z.read(name);assert len(content)==expected['bytes'] and hashlib.sha256(content).hexdigest()==expected['sha256'],name
row.update({'archive':str(p),'zip_crc_verified':True,'every_member_sha256_verified':True,'motion_issued':False,'remote_files_read_only':True})
(HERE/'deployment_evidence_receipt.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
print(json.dumps(row,indent=2))
