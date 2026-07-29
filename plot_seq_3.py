import numpy as np, matplotlib.pyplot as plt

SEQ = 'uzh_of1'   # <-- change this for other sequences

def load_blocks(path):
    blocks, cur = [], []
    for line in open(path):
        if line.strip() == '':
            if cur: blocks.append(np.array(cur)); cur = []
        else:
            cur.append([float(v) for v in line.split()])
    if cur: blocks.append(np.array(cur))
    return blocks

viba = load_blocks(f'result/traj_drtLoosely_{SEQ}.txt')
drt  = load_blocks(f'/home/aze-pc-0331/drt-vio-baseline/result/traj_drtLoosely_{SEQ}.txt')

allb = viba + drt
xs = np.concatenate([np.r_[b[:,0], b[:,2]] for b in allb])
ys = np.concatenate([np.r_[b[:,1], b[:,3]] for b in allb])
xlim = (xs.min()-0.5, xs.max()+0.5); ylim = (ys.min()-0.5, ys.max()+0.5)

def save_plot(blocks, xcol, ycol, color, label, title, fname):
    plt.figure(figsize=(9, 8))
    for b in blocks:
        plt.plot(b[:,xcol], b[:,ycol], color=color, linewidth=0.7)
    plt.plot([], [], color=color, label=label)
    plt.xlabel('x - ekseni (m)'); plt.ylabel('y - ekseni (m)')
    plt.xlim(xlim); plt.ylim(ylim); plt.gca().set_aspect('equal')
    plt.legend(); plt.title(title); plt.tight_layout()
    plt.savefig(fname, dpi=150); plt.close()
    print('saved', fname)

save_plot(viba, 2, 3, 'black', 'ground truth', f'{SEQ} - ground truth', f'result/traj_{SEQ}_gt.png')
save_plot(drt,  0, 1, 'red',   'original DRT', f'{SEQ} - original DRT',  f'result/traj_{SEQ}_baseline.png')
save_plot(viba, 0, 1, 'blue',  'VI-BA',        f'{SEQ} - VI-BA',         f'result/traj_{SEQ}_viba.png')