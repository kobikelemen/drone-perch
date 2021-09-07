import matplotlib.pyplot as plt
import json

f = open('results.txt', 'r')
data = f.readlines()


waypoints = []
poly = []
actual_traj = []
vel_list = []
tracking_error = []

for index, line in enumerate(data):
	
	if 'waypoints' in line:
		waypoints = json.loads(data[index+1])

	elif 'poly' in line:
		poly = json.loads(data[index+1])

	elif 'actual_trajectory' in line:
		actual_traj = json.loads(data[index+1])

	elif 'velocity_list' in line:
		vel_list = json.loads(data[index+1])

	elif 'self.trackingError_list' in line:
		tracking_error = json.loads(data[index+1])


	if len(waypoints) != 0 and len(poly) != 0 and len(actual_traj) != 0 and len(vel_list) != 0 and len(tracking_error) != 0:
		print('--waypoints--', waypoints)
		plt.scatter([i[0] for i in waypoints],[i[1] for i in waypoints], color='blue', label='waypoints')
		plt.plot([i[0] for i in actual_traj ], [i[1] for i in actual_traj], color='red')
		#plt.scatter(x=2.365, y=4.9, color='brown', label='branch')
		plt.plot([i[0] for i in poly], [i[1] for i in poly], color='green', label='poly')
		#plt.quiver([i[0] for i in actual_traj], [i[1] for i in actual_traj], [i[0] for i in vel_list], [i[1] for i in vel_list], scale=1, angles='xy', scale_units='xy', width=0.001)
		#plt.plot([i[0] for i in actual_traj], [i for i in tracking_error], color='orange')
		plt.show()
		waypoint = []
		poly = []
		actual_traj = []
		vel_list = []
		tracking_error = []





