import sys,json,glob,collections
S=sys.argv[1]; tags=sys.argv[2:]
for t in tags:
    rows=[json.loads(l) for f in glob.glob('%s/b_%s_*.txt'%(S,t)) for l in open(f) if l.startswith('{')]
    full=[r for r in rows if r['pts']>=55]
    els=sorted(r.get('el') or 0 for r in full if r.get('el'))
    c=collections.Counter()
    for r in rows: c['pts']+=r['pts']; c['full']+=r['pts']>=55; c['viol']+=r['viol']>0; c['contact']+=r['contact']>0
    print(t,dict(c),'cierre (s) de las perfectas: mediana %.0f, p90 %.0f, max %.0f'%(els[len(els)//2],els[int(len(els)*.9)],els[-1]) if els else '')
