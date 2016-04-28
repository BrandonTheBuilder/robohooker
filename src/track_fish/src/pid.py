class PID(object):
    def __init__(self, kP, kI, kD, outRange):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.outRange = outRange
        # Assuming we are being updated at a constant or near constant rate
        self.err = []
        self.target = 0
        self.out = 0


    def update(self, current):
        err = self.target - current
        self.err.append(float(err))
        P = err*self.kP
        x = len(self.err)
        if x > 1:
            i = sum(self.err)
            d = self.err[x-1]-self.err[x-2]
        else:
            i = 0
            d = 0
        I = self.kI*i
        D = self.kD*d
        out = P+I+D
        # Scaling output to out range
        # out = u*(float(self.outRange[1])/self.inRange[1])
        if out > self.outRange[1]:
            out = self.outRange[1]
        elif out < self.outRange[0]:
            out = self.outRange[0]
        self.out = out
        return out
                 

    def integrate(self, f, x):
        i = 0
        for n in range(len(f) - 1):
            i += f[n]*(x[n+1]-x[n])
        return i
