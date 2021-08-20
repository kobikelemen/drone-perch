import matplotlib.pyplot as plt
import pylab

 

x = pylab.array([i/10 for i in range(0,10)])

y = pylab.array([0, 0.2, 0.3, 0.25, 0.2, 0.15, 0.15, 0.25, 0.45, 0.75])

 

a = pylab.polyfit(x, y, deg=7)
esty = a[0]*(x**7) + a[1]*(x**6) + a[2]*(x**5) + a[3]*(x**4) + a[4]*(x**3) + a[5]*(x**2) +a[6]*(x) + a[7]

plt.plot(x,y)

plt.plot(x,esty)

plt.show()