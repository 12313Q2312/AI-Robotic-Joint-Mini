"""Build an offline review artifact from selected immutable simulator runs."""
from pathlib import Path
import csv
import json
from analyze import analyze,ROOT
from cushion import selected_results

def build():
    config=ROOT/'delivery_cases.json'
    selected=json.loads(config.read_text(encoding='utf-8'))
    cases=[]
    for spec in selected:
        item=analyze(spec['run']);item.update(title=spec['title'],description=spec.get('description',''))
        cases.append(item)
    cushion_dir=selected_results()
    summaries=json.loads((cushion_dir/'summary.json').read_text())
    for key,title in [('cushioned','缓冲对照 · 8厘米行程'),('hard_contact','缓冲对照 · 硬接触'),('insufficient_stroke','失败边界 · 2厘米行程触底')]:
        with (cushion_dir/f'{key}.csv').open() as f:frames=[[round(float(v),7) for v in row] for row in list(csv.reader(f))[1:]][::8]
        cases.append({'type':'cushion','id':key,'title':title,'summary':summaries[key],'frames':frames,'description':'20kg等效质量、0.30m落差；一维部件模型，未与整个人体跌倒耦合。'})
    payload={'schema_version':1,'cases':cases,'evidence_boundary':'synthetic simulation; no human/hardware or clinical validation'}
    (ROOT/'viewer/demo_data.js').write_text('window.MOBILITY_DEMO='+json.dumps(payload,ensure_ascii=False,separators=(',',':'))+';\n',encoding='utf-8')
    (ROOT/'results/selected_runs.json').write_text(json.dumps([x['summary'] for x in cases if x['type']=='walking'],ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'cases':len(cases),'viewer':'mobility_demo/viewer/index.html','bytes':(ROOT/'viewer/demo_data.js').stat().st_size}))
    return payload

if __name__=='__main__':build()
