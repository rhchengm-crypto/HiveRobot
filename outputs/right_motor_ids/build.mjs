import fs from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { FileBlob, SpreadsheetFile } from '@oai/artifact-tool';
const dir = fileURLToPath(new URL('.', import.meta.url));
const wb = await SpreadsheetFile.importXlsx(await FileBlob.load('C:/Users/rhche/Desktop/左臂电机ID备忘.xlsx'));
const sheet = wb.worksheets.getItem('全身参考');
if (process.argv.includes('--preview')) {
  const pic = await wb.render({sheetName:'全身参考',range:'A24:H27',scale:1.5});
  await fs.writeFile(`${dir}/before.png`, new Uint8Array(await pic.arrayBuffer()));
} else {
  for (let i=0;i<3;i++) sheet.getRange(`A${28+i}:H${28+i}`).copyFrom(sheet.getRange(`A${25+i}:H${25+i}`),'all');
  sheet.getRange('A28:H30').values = [
    [27,'右爪/夹爪','待确认','0x1B',27,'0x2B',43,'规划 ID；待写入/测试'],
    [28,'右腕上下','待确认','0x1C',28,'0x2C',44,'规划 ID；待写入/测试'],
    [29,'右腕左右','待确认','0x1D',29,'0x2D',45,'规划 ID；待写入/测试']
  ];
  wb.recalculate();
  console.log((await wb.inspect({kind:'table',range:'全身参考!A28:H30',include:'values,formulas',tableMaxRows:3,tableMaxCols:8})).ndjson);
  const pic=await wb.render({sheetName:'全身参考',range:'A28:H30',scale:1.5});
  await fs.writeFile(`${dir}/after.png`,new Uint8Array(await pic.arrayBuffer()));
  await (await SpreadsheetFile.exportXlsx(wb)).save(`${dir}/左右臂电机ID备忘.xlsx`);
}
