"""Analyze retained server counters; never communicates with hardware."""
import collections
import hashlib
import html
import json
import math
from pathlib import Path
import statistics

OUT = Path(__file__).resolve().parent
records = [json.loads(line) for line in (OUT/'raw.jsonl').read_text().splitlines()]
metadata = json.loads((OUT/'metadata.json').read_text())
history = json.loads((OUT/'previous_session_statistics.json').read_text())
assert metadata.get('complete'), metadata
assert not any(r.get('error') for r in records)
assert all(not r['data']['playing'] for r in records if r['path']=='/api/play')
samples = []
excluded = collections.Counter()
pre = {}
for record in records:
    phase = record['phase']
    if phase.endswith('_pre'):
        pre[phase[:-4]] = record['data']['addresses']
    elif record['path'].endswith('/probe'):
        before = pre[phase]
        for address, bag in record['data']['addresses'].items():
            old = before.get(address, {})
            dn = bag['n']-old.get('n',0)
            dok = bag['ok']-old.get('ok',0)
            assert dn >= 0 and dok >= 0, 'Counters reset during experiment'
            if dn==dok==1:
                value = (bag['rtt_sum_us']-old.get('rtt_sum_us',0))/1000
                assert abs(value-bag['rtt_last_us']/1000)<0.001
                samples.append({'phase':phase,'global_slot':int(address),
                    'ring':'jameson' if int(address)<10 else 'retuned',
                    'local_address':int(address)%10,'rtt_ms':value,'utc':record['utc']})
            else:
                excluded[(phase,f'n_delta={dn},ok_delta={dok}')] += 1


def percentile(values, pct):
    v = sorted(values)
    x=(len(v)-1)*pct/100
    lo=math.floor(x)
    hi=math.ceil(x)
    return v[lo]+(v[hi]-v[lo])*(x-lo)


def stats(values):
    return {'n':len(values),'mean_ms':statistics.mean(values),
            'median_ms':percentile(values,50),'p95_ms':percentile(values,95),
            'p99_ms':percentile(values,99),'min_ms':min(values),'max_ms':max(values),
            'stddev_ms':statistics.pstdev(values)}


summary = {'phases':{},'per_slot':{},'excluded_intervals':
           {f'{p}: {why}':n for (p,why),n in excluded.items()}}
for phase in ['spaced','continuous']:
    for ring in ['jameson','retuned']:
        group=[s['rtt_ms'] for s in samples if s['phase']==phase and s['ring']==ring]
        summary['phases'][f'{phase}/{ring}']=stats(group)
    for address in range(20):
        group=[s['rtt_ms'] for s in samples if s['phase']==phase and s['global_slot']==address]
        summary['per_slot'][f'{phase}/{address}']=stats(group)
before = next(r['data'] for r in records if r['phase']=='before' and r['path']=='/api/bus-health')
after = next(r['data'] for r in records if r['phase']=='after' and r['path']=='/api/bus-health')
summary['counter_delta']={key:sum(b[key]-before['addresses'].get(a,{}).get(key,0)
    for a,b in after['addresses'].items()) for key in ['n','ok','timeouts','crc_errors','errors']}
summary['historical_timing']={k:{'n':v['n'],'mean_ms':v['sum']/v['n'],
    'rms_ms':math.sqrt(v['sumSq']/v['n']),'max_abs_ms':v['maxAbs']}
    for k,v in history['timing_stats'].items()}
res=history['residual_stats']
summary['historical_residual']={'n':sum(v[1] for v in res),
    'mean_ms':sum(v[2] for v in res)/sum(v[1] for v in res),
    'rms_ms':math.sqrt(sum(v[3] for v in res)/sum(v[1] for v in res)),
    'max_abs_ms':max(v[4] for v in res)}
before_play=next(r['data'] for r in records if r['path']=='/api/play')
after_play=next(r['data'] for r in records if r['phase']=='after' and r['path']=='/api/play')
summary['playback_counters_unchanged']=all(before_play[k]==after_play[k] for k in
    ['timing_stats','residual_stats','dispatch_stats'])
assert summary['playback_counters_unchanged']
(OUT/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for ax,phase in zip(axes,['spaced','continuous']):
    for ring,color in [('jameson','#087e8b'),('retuned','#cb6230')]:
        v=[s['rtt_ms'] for s in samples if s['phase']==phase and s['ring']==ring]
        ax.hist(v,bins=[x/2 for x in range(0,81)],histtype='step',linewidth=1.8,
                color=color,label=f'{ring} (n={len(v)})')
    ax.set(title=f'{phase.capitalize()} ring sweeps',xlabel='Server-measured transaction RTT (ms)',ylabel='Samples')
    ax.axvline(16,color='#666666',ls=':',label='16 ms reference')
    ax.legend(fontsize=8)
fig.suptitle('Existing FTDI settings: 16 ms saved latency timer; no motion')
fig.savefig(OUT/'baseline.png',dpi=160)
plt.close(fig)

rows=''.join('<tr><td>'+html.escape(k)+'</td>'+''.join(f'<td>{v[c]:.3f}</td>' if c!='n'
    else f'<td>{v[c]}</td>' for c in ['n','mean_ms','median_ms','p95_ms','p99_ms','max_ms'])+'</tr>'
    for k,v in summary['phases'].items())
hist_rows=''.join(f'<tr><td>{html.escape(k)}</td><td>{v["n"]}</td><td>{v["mean_ms"]:.3f}</td><td>{v["rms_ms"]:.3f}</td><td>{v["max_abs_ms"]:.3f}</td></tr>'
    for k,v in {**summary['historical_timing'],'lag_prediction_residual':summary['historical_residual']}.items())
report=f'''<!doctype html><meta charset="utf-8"><title>Serial ring baseline — 5 October 2026</title>
<style>body{{font:16px system-ui;max-width:1050px;margin:40px auto;padding:0 24px;color:#182738;line-height:1.55}}table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}}th,td{{text-align:right;padding:9px;border-bottom:1px solid #ddd}}th:first-child,td:first-child{{text-align:left}}img{{width:100%}}code{{background:#eee}}</style>
<h1>Serial ring baseline</h1><p>5 October 2026, America/Los_Angeles. Capture: {html.escape(metadata['started_utc'])} through {html.escape(metadata['ended_utc'])} UTC.</p>
<p>Both rings were queried through the existing running HTTPS server. COM29 is jameson (slots 0–9); COM48 is retuned (slots 10–19). Each has ten enumerated actuators. Both FTDI adapters retain saved latency timer 16 ms.</p>
<h2>Measured transaction times</h2><table><tr><th>Phase/ring</th><th>Samples</th><th>Mean ms</th><th>Median ms</th><th>p95 ms</th><th>p99 ms</th><th>Max ms</th></tr>{rows}</table>
<img src="baseline.png" alt="Transaction RTT histograms for both rings, spaced and continuous sweeps">
<p>Counter changes across the full capture: <code>{html.escape(json.dumps(summary['counter_delta']))}</code>. These include initial status reads and any concurrent UI polling. Playback counters were unchanged.</p>
<h2>Method and limits</h2><p>60 two-ring sweeps at one-second start intervals, then 120 back-to-back sweeps. Each ring serializes its ten QUERY_STRIKE transactions, while the two rings run in parallel. Thus even the spaced phase has back-to-back transactions within each sweep; this is not an isolated single-packet or adapter-loopback test.</p>
<p>For every probe, retain the preceding health snapshot and the returned snapshot. A raw RTT sample is admitted only when an address has exactly one additional attempted and successful transaction. Cumulative RTT sum difference must match the returned last RTT within 1 microsecond. Other intervals are excluded from percentile calculations; all raw responses remain available. Exclusions: {html.escape(json.dumps(summary['excluded_intervals']))}.</p>
<p>The reported RTT is the server's own serial transaction duration, including receive draining, write/flush, ring/device processing, USB receive delivery and Python parsing. It excludes HTTP/TLS duration and acquisition of the server bus lock. It does not measure outgoing command latency, synchronized trigger skew, or acoustic impact timing. Existing browser polling was not disabled; admitted one-transaction intervals can include a query initiated by the browser. Current-checkout source hashes are saved, but do not prove the exact code already loaded by the process or flashed into devices.</p>
<h2>Earlier playback statistics preserved before restart</h2><table><tr><th>Metric</th><th>Samples</th><th>Mean ms</th><th>RMS ms</th><th>Max absolute ms</th></tr>{hist_rows}</table>
<p>These are historical session aggregates read before the user stopped and restarted the server. Selected numeric values were transcribed from the API response retained in this conversation. They are not part of the controlled probe sample population and cannot yield percentiles.</p>
<p>Residual means firmware-reported trigger-to-impact lag minus the prediction associated with the prior accepted strike. It excludes transport delay and actual host dispatch lateness. No-ACK chord dispatches do not produce RTT samples and clear residual attribution. The server currently records lock wait and serial RTT, but does not record intended-versus-actual dispatch time for scheduled playback; absence of that metric does not establish absence of scheduler jitter.</p>
<h2>Interpretation and next experiment</h2><p>Concentration near 16 ms is consistent with the configured receive latency timer contributing strongly to turnaround. This is evidence for an A/B experiment, not proof that the entire delay originates inside the adapter. Compare the same non-motion test at a saved 2 ms setting after confirming it is applied by the reopened driver connection. Preserve both datasets. Logic-analyzer or adapter-loopback testing is still needed to isolate wire timing and adapter-only behavior.</p>
<p>No motion commands, enumeration, firmware changes, settings writes, counter resets or server restarts were performed by the capture. The user restarted the server before capture.</p>
<h2>Evidence</h2><p><a href="raw.jsonl">Raw API responses</a> · <a href="samples.json">Admitted samples</a> · <a href="summary.json">Statistics</a> · <a href="metadata.json">Settings and source hashes</a> · <a href="previous_session_statistics.json">Historical values</a> · <a href="capture.py">Capture script</a> · <a href="analyze.py">Analysis script</a></p>'''
(OUT/'report.html').write_text(report,encoding='utf-8')
manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir() if p.is_file() and p.name!='sha256.json'}
(OUT/'sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='per_slot'},indent=2))
