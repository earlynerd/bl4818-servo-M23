"""Offline comparison of retained 16 ms and 2 ms application-probe trials."""
import collections
import hashlib
import html
import json
import math
import statistics
from pathlib import Path

OUT = Path(__file__).resolve().parent
BASELINE = OUT.parent/'serial_baseline_20261005'
records = [json.loads(line) for line in (OUT/'raw.jsonl').read_text().splitlines()]
metadata = json.loads((OUT/'metadata.json').read_text())
baseline_metadata = json.loads((BASELINE/'metadata.json').read_text())
assert metadata.get('complete'), metadata
assert not any(r.get('error') for r in records)
assert all(not r['data']['playing'] for r in records if r['path']=='/api/play')
assert all(a['saved_latency_timer_ms']==2 for a in metadata['adapters'].values())
assert metadata['source_sha256']==baseline_metadata['source_sha256'], 'Source changed between captures'
samples=[]
excluded=collections.Counter()
pre={}
for record in records:
    phase=record['phase']
    if phase.endswith('_pre'):
        pre[phase[:-4]]=record['data']['addresses']
    elif record['path'].endswith('/probe'):
        for address,bag in record['data']['addresses'].items():
            old=pre[phase].get(address,{})
            dn=bag['n']-old.get('n',0)
            dok=bag['ok']-old.get('ok',0)
            assert dn>=0 and dok>=0
            if dn==dok==1:
                value=(bag['rtt_sum_us']-old.get('rtt_sum_us',0))/1000
                assert abs(value-bag['rtt_last_us']/1000)<0.001
                samples.append({'phase':phase,'global_slot':int(address),
                    'ring':'jameson' if int(address)<10 else 'retuned',
                    'local_address':int(address)%10,'rtt_ms':value,'utc':record['utc']})
            else:
                excluded[(phase,f'n_delta={dn},ok_delta={dok}')]+=1


def pct(values,p):
    v=sorted(values)
    x=(len(v)-1)*p/100
    lo=math.floor(x)
    hi=math.ceil(x)
    return v[lo]+(v[hi]-v[lo])*(x-lo)


def stats(v):
    return {'n':len(v),'mean_ms':statistics.mean(v),'median_ms':pct(v,50),
            'p95_ms':pct(v,95),'p99_ms':pct(v,99),'max_ms':max(v),'min_ms':min(v),
            'stddev_ms':statistics.pstdev(v)}


old_summary=json.loads((BASELINE/'summary.json').read_text())
old_samples=json.loads((BASELINE/'samples.json').read_text())
summary={'phases':{},'comparison':{},'per_slot':{},'excluded_intervals':
    {f'{p}: {why}':n for (p,why),n in excluded.items()},'source_hashes_match':True}
for phase in ['spaced','continuous']:
    for ring in ['jameson','retuned']:
        key=f'{phase}/{ring}'
        v=[s['rtt_ms'] for s in samples if s['phase']==phase and s['ring']==ring]
        new=stats(v)
        old=old_summary['phases'][key]
        summary['phases'][key]=new
        summary['comparison'][key]={'before':old,'after':new,
            'median_reduction_percent':100*(1-new['median_ms']/old['median_ms']),
            'median_ratio':old['median_ms']/new['median_ms']}
    for address in range(20):
        summary['per_slot'][f'{phase}/{address}']=stats([s['rtt_ms'] for s in samples
            if s['phase']==phase and s['global_slot']==address])
before=next(r['data'] for r in records if r['phase']=='before' and r['path']=='/api/bus-health')
after=next(r['data'] for r in records if r['phase']=='after' and r['path']=='/api/bus-health')
summary['counter_delta']={key:sum(b[key]-before['addresses'].get(a,{}).get(key,0)
    for a,b in after['addresses'].items()) for key in ['n','ok','timeouts','crc_errors','errors']}
before_play=next(r['data'] for r in records if r['path']=='/api/play')
after_play=next(r['data'] for r in records if r['phase']=='after' and r['path']=='/api/play')
summary['playback_counters_unchanged']=all(before_play[k]==after_play[k]
    for k in ['timing_stats','residual_stats','dispatch_stats'])
assert summary['playback_counters_unchanged']
(OUT/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for ax,ring in zip(axes,['jameson','retuned']):
    for data,color,label in [(old_samples,'#b57237','16 ms setting'),(samples,'#087e8b','2 ms setting')]:
        v=sorted(s['rtt_ms'] for s in data if s['phase']=='continuous' and s['ring']==ring)
        ax.step(v,[(i+1)*100/len(v) for i in range(len(v))],where='post',color=color,label=label,lw=2)
    ax.set(title=ring.capitalize(),xlabel='Serial transaction RTT (ms)',ylabel='Cumulative samples (%)',xlim=(0,40),ylim=(0,101))
    ax.axhline(99,ls=':',color='#aaaaaa')
    ax.grid(alpha=.2)
    ax.legend(loc='lower right')
fig.suptitle('FTDI latency setting A/B — continuous non-motion ring probes')
fig.savefig(OUT/'comparison.png',dpi=170)
plt.close(fig)

rows=''
for key,comparison in summary['comparison'].items():
    a,b=comparison['before'],comparison['after']
    rows+=f'<tr><td>{html.escape(key)}</td><td>{a["n"]}/{b["n"]}</td><td>{a["median_ms"]:.2f} → {b["median_ms"]:.2f}</td><td>{a["p99_ms"]:.2f} → {b["p99_ms"]:.2f}</td><td>{a["max_ms"]:.2f} → {b["max_ms"]:.2f}</td><td>{comparison["median_reduction_percent"]:.1f}%</td></tr>'
report=f'''<!doctype html><meta charset="utf-8"><title>FTDI 16 ms versus 2 ms comparison</title>
<style>body{{font:16px system-ui;max-width:1050px;margin:40px auto;padding:0 24px;color:#182738;line-height:1.55}}table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}}th,td{{text-align:right;padding:9px;border-bottom:1px solid #ddd}}th:first-child,td:first-child{{text-align:left}}img{{width:100%}}code{{background:#eee}}</style>
<h1>FTDI latency timer: 16 ms → 2 ms</h1><p>5 October 2026, America/Los_Angeles. Trial UTC: {html.escape(metadata['started_utc'])} through {html.escape(metadata['ended_utc'])}.</p>
<p>Only COM29 (FTDI A10JO345A, jameson) and COM48 (FTDI BG03STIBA, retuned) were changed. Original 16 ms registry values were saved, both new 2 ms values were read back, and only the idle drum server was restarted with its existing command line. No USB hub reset, driver reinstall, global setting, other port configuration, or other project process was changed. The other enumerated ports were CH344 COM9/11/13/14.</p>
<h2>Measured result</h2><table><tr><th>Phase/ring</th><th>Samples before/after</th><th>Median ms</th><th>p99 ms</th><th>Maximum ms</th><th>Median reduction</th></tr>{rows}</table>
<img src="comparison.png" alt="Cumulative RTT distributions for both rings before and after latency setting change">
<p>Full capture counter delta: <code>{html.escape(json.dumps(summary['counter_delta']))}</code>. Playback counters stayed unchanged. Source-file hashes match the baseline capture.</p>
<h2>Method</h2><p>Same capture code and HTTPS API path as the 16 ms baseline: 60 two-ring status sweeps at one-second start intervals, then 120 back-to-back sweeps. Each ring serializes ten QUERY_STRIKE transactions; rings run in parallel. RTT is measured inside the server and excludes HTTP/TLS and bus-lock acquisition. Exactly-one-success counter intervals yield raw samples, checked against the last RTT value. Other intervals are excluded: {html.escape(json.dumps(summary['excluded_intervals']))}.</p>
<p>Probe durations include host serial draining, write/flush, device forwarding/processing, USB receive delivery, and Python parsing. They are not measurements of outbound strike latency or acoustic impact. No strikes were commanded; the server's startup enumeration only restored its existing ring addresses. Spaced sweeps still contain ten back-to-back transactions per ring. Normal browser polling was not disabled.</p>
<h2>Interpretation</h2><p>The large downward shift after changing the per-device timer supports receive buffering as a substantial contributor to the old transaction turnaround. The remaining delay and outliers include other parts of the stack. This sequential A/B test also includes a drum-server restart and was not randomized or repeated as A/B/A, so it is not a laboratory isolation of every timing source.</p>
<p>Faster replies can release the ring worker sooner for subsequent acknowledged notes and status queries. The gain is not automatically an equal improvement in strike onset or impact accuracy. No-ACK chords already avoid waiting for these replies. Existing learned mallet-lag compensation remains unchanged.</p>
<h2>Current configuration and restoration</h2><p>COM29 and COM48 remain at 2 ms; the drum server is running at the same HTTPS address. The original values and exact adapter identities are in <a href="latency_backup.json">latency_backup.json</a>. The scoped administrator script supports <code>-Mode Restore</code> to restore 16 ms to these two adapters only; reopen the drum connections afterward.</p>
<h2>Evidence</h2><p><a href="raw.jsonl">Raw API snapshots</a> · <a href="samples.json">Individual samples</a> · <a href="summary.json">Detailed statistics</a> · <a href="metadata.json">Capture metadata</a> · <a href="setting_apply_result.json">Registry write/readback</a> · <a href="capture.py">Capture script</a> · <a href="compare.py">Analysis script</a> · <a href="../serial_baseline_20261005/report.html">Original baseline</a></p>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir()
    if p.is_file() and p.name not in ['sha256.json','server_stdout.log','server_stderr.log']}
(OUT/'sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k not in ['per_slot','phases']},indent=2))
