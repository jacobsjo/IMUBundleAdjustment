#test script.
import numpy as np
def read_bal_data(file_name):
    with open(file_name, "r") as file:
        n_cameras, n_points, n_observations = map(int, file.readline().split())
        print ('n_cameras',n_cameras)
        print ('n_points',n_points)
        print ('n_observations',n_observations)

        camera_indices = np.empty(n_observations, dtype=int)
        point_indices = np.empty(n_observations, dtype=int)
        points_2d = np.empty((n_observations, 2))

        for i in range(n_observations):
            camera_index, point_index, x, y = file.readline().split()
            camera_indices[i] = int(camera_index)
            point_indices[i] = int(point_index)
            points_2d[i] = [float(x), float(y)]

        camera_params = np.empty(n_cameras * 9)
        for i in range(n_cameras * 9):
            camera_params[i] = float(file.readline())
            print ('caamera_params',camera_params[i])
        camera_params = camera_params.reshape((n_cameras, -1))
        print ('final camera_params', camera_params)

        points_3d = np.empty(n_points*3)
        for i in range(n_points*3):
            points_3d[i] = float(file.readline())
            #print ('3D Coordinates:',points_3d[i])
        points_3d = points_3d.reshape(n_points,-1)

if __name__ == "__main__":
    file_name = 'test.txt'
    read_bal_data(file_name)
