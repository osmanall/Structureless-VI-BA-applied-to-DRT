import os

SRC = os.path.expanduser('~/Downloads/uzh_fpv')
DST = os.path.join(SRC, 'mav0')
to_ns = lambda t: int(round(float(t) * 1e9))

for d in ['cam0', 'imu0', 'state_groundtruth_estimate0']:
    os.makedirs(os.path.join(DST, d), exist_ok=True)

# images: symlink cam0/data -> img, and write timestamped csv (left cam only)
link = os.path.join(DST, 'cam0', 'data')
if not os.path.exists(link):
    os.symlink(os.path.join(SRC, 'img'), link)
with open(f'{SRC}/left_images.txt') as f, open(f'{DST}/cam0/data.csv', 'w') as o:
    o.write('#timestamp [ns],filename\n')
    for ln in f:
        if ln.startswith('#') or not ln.strip(): continue
        p = ln.split(); o.write(f'{to_ns(p[1])},{os.path.basename(p[2])}\n')

# imu: id timestamp gx gy gz ax ay az  ->  ts,gx,gy,gz,ax,ay,az
with open(f'{SRC}/imu.txt') as f, open(f'{DST}/imu0/data.csv', 'w') as o:
    o.write('#timestamp [ns],wx,wy,wz,ax,ay,az\n')
    for ln in f:
        if ln.startswith('#') or not ln.strip(): continue
        p = ln.split(); o.write(f'{to_ns(p[1])},{p[2]},{p[3]},{p[4]},{p[5]},{p[6]},{p[7]}\n')

# gt: timestamp tx ty tz qx qy qz qw  (no vel/bias)
gt = []
with open(f'{SRC}/groundtruth.txt') as f:
    for ln in f:
        if ln.startswith('#') or not ln.strip(): continue
        p = ln.split()
        gt.append((float(p[0]), list(map(float, p[1:4])), list(map(float, p[4:8]))))  # qx qy qz qw
with open(f'{DST}/state_groundtruth_estimate0/data.csv', 'w') as o:
    o.write('#ts,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz\n')
    for i, (t, pos, (qx, qy, qz, qw)) in enumerate(gt):
        if 0 < i < len(gt) - 1:                          # central-difference velocity
            t0, p0, _ = gt[i-1]; t1, p1, _ = gt[i+1]; dt = t1 - t0
            v = [(p1[k]-p0[k])/dt for k in range(3)] if dt > 0 else [0,0,0]
        else:
            v = [0, 0, 0]
        o.write(f'{to_ns(t)},{pos[0]},{pos[1]},{pos[2]},{qw},{qx},{qy},{qz},'
                f'{v[0]},{v[1]},{v[2]},0,0,0,0,0,0\n')
print('done ->', DST)