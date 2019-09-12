import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def analyze_waypoints(data_fname):
    df = pd.read_csv(data_fname, header=None)
    N = len(df)
    
    p0 = np.array(df.iloc[0][:2])
    p1 = np.array(df.iloc[1][:2])
    
    way_points = []
    way_points.append(p0)
    way_points.append(p1)

    prev = p1 - p0
    prev /= np.linalg.norm(prev)
    
    heading_angles = []
    for n in range(2, N):
        p0 = np.copy(p1)
        p1 = np.array(df.iloc[n][:2])
        way_points.append(p1)

        cur = p1 - p0   
        cur /= np.linalg.norm(cur)

        val = np.dot(prev, cur)
        val = max(-1.0, min(1.0, val))
        if val > 1 or val < -1:
            check = True
        theta = np.arccos(val) * 180/np.pi
        heading_angles.append(theta)

        prev = np.copy(cur)

    way_points = np.array(way_points)
    return heading_angles, way_points

if __name__ == '__main__':
    heading_angles, way_points = analyze_waypoints('../../../data/wp_yaw_const.csv')
    
    plt.plot(way_points[:, 0], way_points[:, 1], '.', color='red')
    plt.show()

    plt.plot(heading_angles)
    plt.show()

