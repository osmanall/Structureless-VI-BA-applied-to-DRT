import numpy as np, matplotlib.pyplot as plt

for k in ['MH01','MH02','MH03','MH04','MH05']:
    blocks, cur = [], []
    for line in open(f'result/traj_drtLoosely_{k}.txt'):
        if line.strip() == '':
            if cur: blocks.append(np.array(cur)); cur = []
        else:
            cur.append([float(v) for v in line.split()])
    if cur: blocks.append(np.array(cur))

    plt.figure(figsize=(11, 8))
    for b in blocks:
        plt.plot(b[:,2], b[:,3], color='black', linewidth=0.7)
        plt.plot(b[:,0], b[:,1], color='blue',  linewidth=0.7)
    plt.plot([], [], 'k', label='ground truth')
    plt.plot([], [], 'b', label='VI-BA estimate')
    plt.xlabel('x - ekseni (m)'); plt.ylabel('y - ekseni (m)')
    plt.axis('equal'); plt.legend(); plt.title(k); plt.tight_layout()
    plt.savefig(f'result/traj_{k}.png', dpi=150); plt.close()
    print(f'saved result/traj_{k}.png')