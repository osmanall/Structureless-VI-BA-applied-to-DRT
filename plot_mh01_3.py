import numpy as np, matplotlib.pyplot as plt

def load_blocks(path):
    blocks, cur = [], []
    for line in open(path):
        if line.strip() == '':
            if cur: blocks.append(np.array(cur)); cur = []
        else:
            cur.append([float(v) for v in line.split()])
    if cur: blocks.append(np.array(cur))
    return blocks

viba = load_blocks('result/traj_drtLoosely_MH01.txt')
drt  = load_blocks('/home/aze-pc-0331/drt-vio-baseline/result/traj_drtLoosely_MH01.txt')

# shared axis limits so all 3 plots line up (from every x/y across both files)
allb = viba + drt
xs = np.concatenate([np.r_[b[:,0], b[:,2]] for b in allb])
ys = np.concatenate([np.r_[b[:,1], b[:,3]] for b in allb])
mx, my = 0.5, 0.5   # margin
xlim = (xs.min()-mx, xs.max()+mx); ylim = (ys.min()-my, ys.max()+my)

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

# 1) ground truth only  (cols 2,3)
save_plot(viba, 2, 3, 'black', 'ground truth', 'MH01 - ground truth',  'result/traj_MH01_gt.png')
# 2) baseline original DRT estimate  (cols 0,1)
save_plot(drt,  0, 1, 'red',   'original DRT', 'MH01 - original DRT',   'result/traj_MH01_baseline.png')
# 3) VI-BA estimate  (cols 0,1)
save_plot(viba, 0, 1, 'blue',  'VI-BA',        'MH01 - VI-BA',          'result/traj_MH01_viba.png')