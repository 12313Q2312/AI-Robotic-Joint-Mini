"""Generate the acceptance artifact from fixed run IDs and explicit predicates."""
from pathlib import Path
import csv
import hashlib
import html
import json
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze import analyze,compare_runs,ROOT
from cushion import selected_results

def render_markdown(text):
    out=[];code=False;table=False;listing=False
    def inline(s):
        s=html.escape(s)
        s=re.sub(r'\[([^]]+)\]\(([^)]+)\)',r'<a href="\2">\1</a>',s)
        s=re.sub(r'`([^`]+)`',r'<code>\1</code>',s)
        return re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',s)
    for line in text.splitlines():
        if line.startswith('```'):
            out.append('</pre>' if code else '<pre>');code=not code;continue
        if code:out.append(html.escape(line)+'\n');continue
        if not line.startswith('|') and table:out.append('</tbody></table>');table=False
        if not line.startswith('- ') and listing:out.append('</ul>');listing=False
        if line.startswith('|'):
            cells=[c.strip() for c in line.strip('|').split('|')]
            if all(re.fullmatch(r'[: -]+',c) for c in cells):continue
            if not table:out.append('<table><tbody>');table=True
            out.append('<tr>'+''.join('<td>'+inline(c)+'</td>' for c in cells)+'</tr>')
        elif line.startswith('#'):
            level=len(line)-len(line.lstrip('#'));out.append(f'<h{level}>'+inline(line[level:].strip())+f'</h{level}>')
        elif line.startswith('- '):
            if not listing:out.append('<ul>');listing=True
            out.append('<li>'+inline(line[2:])+'</li>')
        elif line.strip():out.append('<p>'+inline(line)+'</p>')
    if table:out.append('</tbody></table>')
    if listing:out.append('</ul>')
    style='body{font:15px/1.9 "Segoe UI","Microsoft YaHei",sans-serif;color:#243c48;max-width:1100px;margin:45px auto;padding:0 30px;background:#f8fafb}h1{font-size:30px}h2{margin-top:38px;border-bottom:1px solid #dae5e9;padding-bottom:8px}table{border-collapse:collapse;width:100%;font-size:12px}td{border:1px solid #d8e2e7;padding:10px}tr:first-child{background:#e9f0f2;font-weight:bold}a{color:#087f73}code,pre{background:#e9eff2;padding:3px 5px;border-radius:4px}pre{white-space:pre-wrap;padding:18px}img{max-width:100%}'
    return '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>仿真验收报告</title><style>'+style+'</style><body>'+''.join(out)+'</body></html>'

def generate():
    specs=json.loads((ROOT/'delivery_cases.json').read_text(encoding='utf-8'))
    values={s['run']:analyze(s['run'])['summary'] for s in specs}
    off=values['validation_flat_off_04'];on=values['validation_flat_assist_04'];bare=values['validation_flat_no_device_04'];stairs=values['validation_stairs_04'];small=values['validation_push_small_04'];large=values['validation_push_large_04'];stop=values['validation_stop_04']
    cushion_dir=selected_results();cushion=json.loads((cushion_dir/'summary.json').read_text())
    reduced=1-on['human_equivalent_positive_work_j']/off['human_equivalent_positive_work_j'];bare_reduced=1-on['human_equivalent_positive_work_j']/bare['human_equivalent_positive_work_j']
    pair=compare_runs(off['run'],on['run']);source_same=pair['valid']
    distance_difference=abs(on['forward_distance_m']/off['forward_distance_m']-1)
    flat_ok=on['completed'] and on['alternating_touchdowns'] and on['stable_touchdown_count']>=6 and on['min_hip_height_m']>=.65 and on['max_trunk_tilt_deg']<=20 and on['last_2s_pelvis_range_m']<=.02 and on['last_2s_filtered_speed_p95_m_s']<=.03 and on['last_2s_both_feet_contact_fraction']>=.98
    stair_levels=all(any(abs(e['ankle_height_m']-(.08+.1*level))<=.025 and e['side']==side for e in stairs['touchdowns']) for level in (1,2,3) for side in ('r','l'))
    stairs_ok=stairs['completed'] and stairs['alternating_touchdowns'] and stair_levels and min(stairs['final_ankle_heights_m'])>=.36 and stairs['max_trunk_tilt_deg']<=20 and stairs['last_2s_pelvis_range_m']<=.02 and stairs['last_2s_filtered_speed_p95_m_s']<=.03 and stairs['last_2s_both_feet_contact_fraction']>=.98
    effort_ok=source_same and distance_difference<=.05 and reduced>=.10 and off['completed'] and on['completed']
    admission_ok=on['rejected_cloud_candidates']==3 and on['local_samples_after_virtual_cloud_offline']>0
    buffer=cushion['cushioned'];buffer_ok=not buffer['bottomed_out'] and cushion['comparison']['peak_force_reduction_fraction']>=.15 and buffer['max_energy_residual_fraction']<.01
    qa_path=ROOT/'results/viewer_qa.json';qa=json.loads(qa_path.read_text()) if qa_path.exists() else {'passed':False,'status':'pending'}
    decision='整体未通过；本轮交付为可分项验收的研发原型' if not stairs_ok else '指定仿真工况可分项验收；全身保护与真人效果仍未验证'
    rows=[('A1','浮动基座与直接力矩','通过（指定模型）','正常行走无托举、重定位；外力仅在扰动试验施加'),
          ('A2','六次迈步、停稳','通过' if flat_ok else '未通过',f"{on['stable_touchdown_count']}次稳定落脚，{on['duration_s']:.1f}s；末2s骨盆范围{on['last_2s_pelvis_range_m']*1000:.3f}mm"),
          ('A3','三阶上楼','通过' if stairs_ok else '未通过',f"{stairs['duration_s']:.3f}s结束，{stairs['stable_touchdown_count']}次稳定落脚，最大躯干倾角{stairs['max_trunk_tilt_deg']:.1f}°"),
          ('A4','人体等效正功降低≥10%','通过（代理指标）' if effort_ok else '未通过',f"同质量基线降低{reduced*100:.2f}%；距离差{distance_difference*100:.2f}%；不是代谢或肌电"),
          ('A5','有界助力、候选计划检查','接口范围内通过' if admission_ok else '未通过','3类候选拒绝；模拟云端离线后本地完成有限任务，非完整物理可行性证明'),
          ('A6','小扰动与大扰动边界','指定试验已记录',f"小扰动完成={small['completed']}；大扰动完成={large['completed']}，不宣称已防止跌倒"),
          ('A7','有限行程缓冲','部件模型通过' if buffer_ok else '部件模型未通过','一维20kg/0.30m，未接入三维人体碰撞或部署装置'),
          ('A8','回放与验收材料','通过' if qa.get('passed') else '待浏览器检查','真实状态回放、原始日志、源码指纹、失败与实现说明')]
    warning_text='未触发' if on.get('first_warning_s') is None else f"{on['first_warning_s']:.3f}秒"
    prepare_text='未触发' if on.get('first_buffer_prepare_signal_s') is None else f"{on['first_buffer_prepare_signal_s']:.3f}秒"
    text=f'''# 人机协同助力仿真验收报告 v0.1

记录：2026-11-04—2026-11-05（Asia/Shanghai）。助手自测报告，**等待用户验收**。

**{decision}。**

本轮做出了可运行的合成人体助力仿真、平地/三阶上楼、实际状态回放、独立缓冲模型和证据记录。三阶上楼与全身跌倒缓冲分别按实际状态处理，不能把部分通过写成整个项目完成。没有真人、真实硬件或临床验证。

## 建议怎样验收

先打开[演示](viewer/index.html)，依次看助力开启、关闭、不穿设备、途中停止、小扰动、大扰动和上楼。拖动时间轴查看实际落脚和失稳，再看缓冲对照中的8cm行程与2cm触底。最后核对下表，记录接受、退回与需补测项。

## 逐项结果

| 编号 | 目标 | 助手自测结论 | 证据与边界 |
|---|---|---|---|
'''+''.join('| '+' | '.join(r)+' |\n' for r in rows)+f'''
停稳量化使用末2秒骨盆平面位移范围≤20mm、0.1秒位移差分速度95分位≤0.03m/s，并检查双脚接触。该工程细化不属于临床稳定标准；原始范围和速度同时保留，不能只看通过标签。

## 助力是否真的分担了负担

| 工况 | 人体+设备质量 | 实际前进 | 人体等效正功 | 设备机械正功 |
|---|---|---|---|---|
| 不穿设备 | 70.24kg | {bare['forward_distance_m']:.4f}m | {bare['human_equivalent_positive_work_j']:.2f}J | 0J |
| 穿设备但不助力 | 74.24kg | {off['forward_distance_m']:.4f}m | {off['human_equivalent_positive_work_j']:.2f}J | 0J |
| 开启助力 | 74.24kg | {on['forward_distance_m']:.4f}m | {on['human_equivalent_positive_work_j']:.2f}J | {on['device_positive_mechanical_work_j']:.2f}J |

同质量对照的人体等效正功减少{reduced*100:.2f}%，相对不穿设备减少{bare_reduced*100:.2f}%。两次同质量任务分别为{off['duration_s']:.1f}/{on['duration_s']:.1f}秒。比较器核对非空且完整的源码指纹、同一物理世界、质量、任务/模式、时长和交替落脚；综合条件通过={source_same}。骨盆轨迹RMS差{pair['root_position_rms_m']*1000:.2f}mm，关节角RMS差{pair['joint_angle_rms_rad']:.4f}rad。工程比较阈值为骨盆RMS≤50mm、关节RMS≤0.10rad，并非临床等效性标准。

功按照每个区间保持的上一周期力矩乘以实际角位移累积，不把下一周期命令与上一周期角速度错配。它不是省电率、代谢降低或真人疲劳改善。

## 云端、本地与危险响应

在5、12、14秒分别注入过期、阶段不匹配、超出请求能力的模拟候选，并拒绝。15秒后模拟云端离线，本地仍采集{on['local_samples_after_virtual_cloud_offline']}个控制样本，完成已接纳的有限任务并停止。没有真实互联网故障或大模型服务调用。

正常助力工况首次告警：{warning_text}；首次缓冲准备信号：{prepare_text}。早期02版正常落脚有短脉冲误报，03版加入80ms持续条件后重跑。这个持续条件有额外时延，只能说明本组回归结果，不能声称真实老人危险识别率。

小扰动是12.8秒起60N、0.15秒；大扰动是400N、0.35秒。大扰动试验在{large['duration_s']:.3f}秒达到失稳结束条件。结束条件不是撞地时刻，因此不能据此计算或宣称碰撞前的部署提前量。缓冲准备只是一条信号，并未让人体获得实际缓冲保护。

## 缓冲容量与失败

| 工况 | 峰值接触力 | 最大压缩 | 触底 |
|---|---|---|---|
'''+''.join(f"| {label} | {cushion[key]['peak_contact_force_n']:.1f}N | {cushion[key]['max_compression_m']*100:.3f}cm | {cushion[key]['bottomed_out']} |\n" for key,label in [('hard_contact','硬接触'),('cushioned','8cm缓冲'),('insufficient_stroke','2cm缓冲')])+f'''
指定一维模型的峰值下降{cushion['comparison']['peak_force_reduction_fraction']*100:.2f}%，8cm缓冲未触底。硬接触以有限刚度表示，不是无限刚性数学冲击；其“触底”字段表示零缓冲行程后的硬接触，不是缓冲装置失效。2cm缓冲触底是明确失败工况。

8cm工况最大能量平衡相对误差{buffer['max_energy_residual_fraction']:.3g}。较小时间步的收敛检查另存测试结果。上述参数是假设，不是材料选型、人体保护范围或减伤结论。

## 验证边界与下一轮工作

- 三阶目标是否通过={stairs_ok}。未通过时优先解决台阶承重过渡、可达性和姿态稳定，不能交付为可供老人使用的上楼功能。
- 人体与设备是合并关节的力矩叠加模型，尚无肌肉骨骼、软组织、绑带和意图估计误差验证。
- 步态缓慢分阶段，尚不能验证接近年轻人的自然行走体验。
- 危险判别仅为带时间条件的规则基线；单个大扰动和同组正常回归不构成全面误报、漏报或保护边界评价。
- 缓冲仍是独立一维模型；下一步要接入全身碰撞、保护部位、部署时延、有限容量和二次碰撞，而不是把准备信号当作成功保护。

## 上楼修正与参数代价

04版在每次迈步开始时，从仿真器取得实际脚位与骨盆水平位置重新接续参考，并逐步回到标称脚间距。这只更新控制目标，没有重置或搬动模型。上楼助力同时从35Nm/0.35增益调整为80Nm/0.65，变化率350Nm/s，并加入髋侧摆助力；平地仍为35Nm/0.35、250Nm/s。上楼配置与旧03版本同时改变了能力和接续机制，不能凭最终通过把收益全部归于某一项改进。

当前4kg设备质量和80Nm峰值能力只是仿真假设，尚未做电机、减速器、电池、散热及峰值持续时间的机械可行性闭合。人体主动控制器也不是经老年人力量数据标定的模型。当前结果证明指定合成工况能运行，尚不证明老人可用、舒适或省力；没有上楼助力关闭的成功配对证据，因此不报告上楼减负百分比。

历史03版三阶失败仍展示。正式04版重新运行全部七个物理工况，并以各次运行的源码指纹区分版本。

## 数据与复现

'''+''.join(f"- `{s['run']}`：[CSV](evidence/runs/{s['run']}.csv)、[事件](evidence/runs/{s['run']}.events.jsonl)、[运行指纹](evidence/runs/{s['run']}.json)。\n" for s in specs)+'''
完整验收证据在[版本化证据包](evidence_bundle_v02.zip)，指纹见[evidence_bundle.manifest.json](evidence_bundle.manifest.json)。启动演示时会验证并解包；如果已有文件内容不同，程序拒绝覆盖。包内只含合成工程数据，公开运行摘要已去除本机启动输出中的个人路径。较早失败的完整日志仍单独保留在本地，没有删除。
每次运行使用新编号，不覆盖前次证据。运行JSON有源码/头文件/世界SHA256；较早原型缺失的历史指纹没有补造。全部失败保留在本地runs目录。未因文档改动重跑无关的历史ROS 2或CAD验收。

详细假设、控制与信号含义见[模型与实现说明](MODEL_AND_IMPLEMENTATION.html)。

## 用户验收记录

- 接受的分项：待用户填写。
- 退回与补测要求：待用户填写。
- 是否进入下一阶段：待用户决定。助手没有代替用户签署验收。
'''
    (ROOT/'ACCEPTANCE_REPORT.md').write_text(text,encoding='utf-8')
    (ROOT/'ACCEPTANCE_REPORT.html').write_text(render_markdown(text),encoding='utf-8')
    (ROOT/'MODEL_AND_IMPLEMENTATION.html').write_text(render_markdown((ROOT/'MODEL_AND_IMPLEMENTATION.md').read_text(encoding='utf-8')),encoding='utf-8')
    (ROOT/'results/acceptance_metrics.json').write_text(json.dumps(list(values.values()),indent=2),encoding='utf-8')
    (ROOT/'results/acceptance_decision.json').write_text(json.dumps({'decision':decision,'flat_pass':flat_ok,'stairs_pass':stairs_ok,'effort_pass':effort_ok,'admission_pass':admission_ok,'component_buffer_pass':buffer_ok,'viewer_qa_pass':qa.get('passed',False),'user_acceptance':'pending'},ensure_ascii=False,indent=2),encoding='utf-8')
    out=ROOT/'results/plots';out.mkdir(exist_ok=True)
    fig,ax=plt.subplots(figsize=(8,3.8));ax.bar(['No device','Device off','Assistance on'],[bare['human_equivalent_positive_work_j'],off['human_equivalent_positive_work_j'],on['human_equivalent_positive_work_j']],color=['#8fa3b0','#b6bdc3','#2a9f8a']);ax.set_ylabel('Synthetic human positive work (J)');ax.set_title('Same task; proxy mechanical work, not metabolic cost');fig.tight_layout();fig.savefig(out/'work_comparison.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,3.8))
    for key,color in [('hard_contact','#b86855'),('cushioned','#2a9f8a'),('insufficient_stroke','#c2a04f')]:
        with (cushion_dir/f'{key}.csv').open() as f:points=list(csv.DictReader(f))
        ax.plot([float(x['time_s']) for x in points],[float(x['contact_force_n'])/1000 for x in points],label=key,color=color)
    ax.set(xlabel='Time (s)',ylabel='Contact force (kN)',xlim=(.22,.5),title='20 kg / 0.30 m: independent component model');ax.legend();fig.tight_layout();fig.savefig(out/'cushion_comparison.png',dpi=160);plt.close(fig)
    print(json.dumps({'decision':decision,'flat':flat_ok,'stairs':stairs_ok,'work_reduction':reduced,'viewer_qa':qa.get('passed',False)},ensure_ascii=True))

if __name__=='__main__':generate()
