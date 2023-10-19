%matplotlib inline
from tclab import clock, setup, Historian, Plotter


def PID(Kp, Ki, Kd, MV_bar=0):
    e_prev = 0
    t_prev = -100
    I = 0

    MV = MV_bar

    while True:
        t, PV, SP = yield MV

        e = SP - PV

        P = Kp*e
        I = I + Ki*e*(t - t_prev)
        D = Kd*(e - e_prev)/(t - t_prev)

        MV = MV_bar + P + I + D

        e_prev = e
        t_prev = t
      
TCLab = setup(connected=False, speedup=10)      
controller = PID(5, 1, 3)
controller.send(None)
tfinal = 800

with TCLab() as lab:
    h = Historian([('SP', lambda: SP), ('T1', lambda: lab.T1), ('MV', lambda: MV), ('Q1', lab.Q1)])
    p = Plotter(h, tfinal)
    T1 = lab.T1
    for t in clock(tfinal, 2):
        SP = T1 if t < 80 else 80
        PV = lab.T1
        MV = controller.send([t, PV, SP])
        lab.U1 = MV
        p.update(t)
