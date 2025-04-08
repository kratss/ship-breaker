# we have a point xy
# its on a line y=mx+b
# if we draw the line c=-mx+y, we get a line with all possible parameeters
import numpy as np
import matplotlib.pyplot as plt
import random
import sklearn.cluster import KMeans

def gen_integers(n, min_val, max_val):
    return [random.randint(min_val, max_val) for _ in range(n)]


points = np.array([gen_integers(50, 0, 100), gen_integers(50, 0, 100)])
points = np.transpose(points)
print(points[0])

plt.scatter(points[:, 0], points[:, 1])
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Points")
plt.show()

plt.plot(points[0, 0], points[0, 1])

# hough space
m = points[0, 0]  # slope
b = points[0, 1]  # y-intercept
x = np.linspace(0, 100, 2)  # you can choose your own range and number of points
y = m * x + b

# Create the plot
plt.figure(figsize=(8, 6))
plt.plot(x, y, label=f"y = {m}x + {b}")
plt.xlabel("x")
plt.ylabel("y")
plt.title("Line Plot: y = m*x + b")
plt.axhline(0, color="gray", linewidth=0.5)  # x-axis
plt.axvline(0, color="gray", linewidth=0.5)  # y-axis
plt.legend()
plt.grid(True)
plt.show()


def getIntersection(plane, cloud):
    cloud_intersect = cloud
    for i in range len(cloud_intersect):
        if a*cloud_intersect[i,0] + \
           b*cloud_intersect[i,1] + \
           c*cloud_intersect[i,2] != 0: 
            #TODO: optimize & tolerate a bit of deviation
            cloud_intersect[i,4]=True

def edgeDetection(cloud):
    # k nearest neighbor
    # k clusters 
    # initialize centroids
    

def main():
    #read cloud from file as [x,y,z,tag]
    #read plane definition from file as [a,b,c] as in ax + by + cz = 0

    # Add a column to hold a tag for whether the point intersects with the cutting plane
    zeros_column = np.zeros((arr.shape[0], 1))
    cloud = np.hstack((cloud, zeros_column))

    # Tag cloud with interesection information
    cloud = getIntersection(plane, cloud)

    # Perform k means to identify nearby points
    # 

if __name__ == '__main__':
    main()
