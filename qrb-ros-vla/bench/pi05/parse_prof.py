import re,sys,subprocess,json
def parse(log):
    try: t=subprocess.run(['qnn-profile-viewer','--input_log',log],capture_output=True,text=True,timeout=120).stdout
    except Exception as e: return {'err':str(e)}
    r={}
    m=re.search(r'Init Stats:.*?NetRun:\s+(\d+) us',t,re.S);  r['init_ms']=int(m.group(1))/1000 if m else None
    sec=t.split('Execute Stats (Average):')
    if len(sec)>1:
        m=re.search(r'NetRun:\s+(\d+) us',sec[1]);            r['exec_avg_ms']=int(m.group(1))/1000 if m else None
        m=re.search(r'Accelerator \(execute\) time\):\s+(\d+) us',sec[1]); r['accel_ms']=int(m.group(1))/1000 if m else None
        m=re.search(r'HVX threads used\):\s+(\d+)',sec[1]);    r['hvx']=int(m.group(1)) if m else None
    sec=t.split('Execute Stats (Min):')
    if len(sec)>1:
        m=re.search(r'NetRun:\s+(\d+) us',sec[1]);            r['exec_min_ms']=int(m.group(1))/1000 if m else None
    m=re.search(r'IPS \(includes IO and misc\. time\):\s+([\d.]+)',t); r['ips']=float(m.group(1)) if m else None
    return r
if __name__=='__main__': print(json.dumps(parse(sys.argv[1])))
