
def func(x):
	return x**3 - x**2 - x

points = []
for p in range(10):
	points.append(round(func(p * 20/100), 3))
print(points)