import matplotlib.pyplot as plt
import json

f = open('results.txt', 'r')
data = f.readlines()

def show_results():

	waypoints = []
	poly = []
	actual_traj = []
	vel_list = []
	tracking_error = []

	drone_stationary_start = 0.77

	for index, line in enumerate(data):
		
		if 'waypoints' in line:
			print(data[index+1])
			waypoints = json.loads(data[index+1])

		elif 'poly' in line:
			poly = json.loads(data[index+1])

		elif 'actual_trajectory' in line:
			actual_traj = json.loads(data[index+1])

		elif 'velocity_list' in line:
			vel_list = json.loads(data[index+1])

		elif 'self.trackingError_list' in line:
			tracking_error = json.loads(data[index+1])

		if len(waypoints) != 0  and len(actual_traj) != 0 and len(poly) != 0:
			
			pl_x = []
			pl_y = []
			for j in waypoints:
				pl_x += [j[0], j[3], j[6], j[9]]
				pl_y += [j[1], j[4], j[7], j[10]]
			plt.scatter(pl_x, pl_y, color='blue', label='waypoints')
			plt.plot([i[0] for i in actual_traj ], [i[1] for i in actual_traj], color='red')
			plt.scatter(x=2.762+0.089, y=4.78-3.361-drone_stationary_start, color='brown', label='branch')
			for j in poly:
				plt.plot([i[0] for i in j], [i[1] for i in j], color='green', label='poly')
			plt.show()
			waypoint = []
			poly = []
			actual_traj = []
			vel_list = []
			tracking_error = []



if __name__ == '__main__':
	show_results()


