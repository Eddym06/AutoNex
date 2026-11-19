import sys,json,glob,collections
S=sys.argv[1]; tags=sys.argv[2:]
data={}
for t in tags:
    rows=[json.loads(l) for f in glob.glob('%s/b_%s_*.txt'%(S,t)) for l in open(f) if l.startswith('{')]
    data[t]={(r['seed'],r['dir']):r for r in rows}
for t,d in data.items():
    c=collections.Counter()
    for r in d.values():
        c['n']+=1; c['pts']+=r['pts']; c['full']+=r['pts']>=55; c['viol']+=r['viol']>0; c['contact']+=r['contact']>0; c['stall']+=r['end']=='stalled'
    print(t,dict(c))
if len(tags)>=2:
    a,b=data[tags[0]],data[tags[1]]
    diff=[(k,a[k]['pts'],b[k]['pts']) for k in sorted(a) if k in b and (a[k]['pts']!=b[k]['pts'])]
    print('distintas',len(diff),diff[:20])
