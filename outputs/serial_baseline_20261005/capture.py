"""Non-motion baseline through the already-running application's HTTPS API.

No serial opens, counter resets, parameter changes, or motion endpoints.
Raw responses are retained so concurrency and counter deltas can be audited.
"""
import datetime as dt
import hashlib
import json
import pathlib
import ssl
import subprocess
import time
import urllib.request
import winreg

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = pathlib.Path(__file__).resolve().parent
BASE = 'https://192.168.8.109:8766'
CONTEXT = ssl.create_default_context(cafile=str(ROOT / 'certs/player.pem'))
ALLOWED = {'/api/play', '/api/bus-health', '/api/strike-timing',
           '/api/status', '/api/firmware', '/api/bus-health/probe'}
RAW = (OUT / 'raw.jsonl').open('x', encoding='utf-8')


def request(path, phase):
    assert path in ALLOWED
    started = time.perf_counter_ns()
    record = {'utc': dt.datetime.now(dt.timezone.utc).isoformat(),
              'phase': phase, 'path': path, 'start_perf_ns': started}
    try:
        req = urllib.request.Request(BASE + path,
            method='POST' if path.endswith('/probe') else 'GET')
        with urllib.request.urlopen(req, context=CONTEXT, timeout=15) as response:
            record['data'] = json.load(response)
        return record['data']
    except Exception as exc:
        record['error'] = repr(exc)
        raise
    finally:
        record['http_ms'] = (time.perf_counter_ns() - started) / 1e6
        RAW.write(json.dumps(record) + '\n')
        RAW.flush()


def idle(phase):
    if request('/api/play', phase)['playing']:
        raise RuntimeError('Playback became active; baseline stopped without stopping playback')


metadata = {'base': BASE, 'started_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
            'method': 'Existing server probes; normal browser polling may coexist',
            'git_head': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'source_sha256': {}, 'adapters': {}}
for name in ['scripts/ring_bus.py','scripts/ring_midi_server.py','scripts/ring_fleet.py',
             'src/protocol.c','src/app_uart.c','player/midi_player.html']:
    metadata['source_sha256'][name] = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'SYSTEM\CurrentControlSet\Enum\FTDIBUS') as root:
    for i in range(winreg.QueryInfoKey(root)[0]):
        name = winreg.EnumKey(root, i)
        if not any(s in name for s in ['A10JO345','BG03STIB']):
            continue
        with winreg.OpenKey(root, name + r'\0000\Device Parameters') as key:
            port = winreg.QueryValueEx(key, 'PortName')[0]
            metadata['adapters'][port] = {'device_key': name,
                'saved_latency_timer_ms': winreg.QueryValueEx(key, 'LatencyTimer')[0]}
(OUT/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')

try:
    idle('before')
    for path in ['/api/bus-health','/api/strike-timing','/api/firmware','/api/status']:
        value = request(path,'before')
        (OUT/('before_'+path.rsplit('/',1)[1]+'.json')).write_text(json.dumps(value,indent=2),encoding='utf-8')
    request('/api/play','before_saved')
    for phase, count, period in [('spaced',60,1.0),('continuous',120,0.0)]:
        start = time.monotonic()
        for i in range(count):
            idle(phase)
            request('/api/bus-health',phase+'_pre')
            request('/api/bus-health/probe',phase)
            if (i+1)%20 == 0:
                print(json.dumps({'phase':phase,'sweeps':i+1,'elapsed_s':round(time.monotonic()-start,2)}),flush=True)
            delay = start+(i+1)*period-time.monotonic()
            if delay>0:
                time.sleep(delay)
    for path in ['/api/play','/api/bus-health','/api/strike-timing']:
        request(path,'after')
    metadata['complete'] = True
except Exception as exc:
    metadata['complete'] = False
    metadata['error'] = repr(exc)
    raise
finally:
    RAW.close()
    metadata['ended_utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
