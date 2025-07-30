# Path Planning for Ship Breaking 
This program takes a segmented point cloud and desired cutting plane, and returns an ordered list of points that create a cutting path for the cutting robot to follow with a plasma torch.

## Setup
Download the repository and prepare the Python environment
```
git clone https://github.com/kratss/ship-breaker.git 
cd ship-breaker
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
chmod u+x *.py
```

An X environment is required to see the 3D visualizations. A Wayland environment is required for the 2D visualizations

## Usage
This example code is also found in demo.py. Simply run `./demo.py`

Import the required libraries
```
import contour
from icecream import ic
import model
import open3d as o3d
import path
```

Specify parameters
```
DENSITY = 65  # Only for synthetic data generation
NOISE_STD = 0.05 # Only for synthetic data generation
Z_PLANE = 5.1  # Location of the cutting plane along the z axis
GRID_DENSITY = 10  # Pixels in the image space per unit length in point cloud space
TOLERANCE = 1.1 # Amount of deviation from the cutting plane permitted
```

The program expects a dictionary containing the clouds such that the key is the component label and the value is the n x 3 numpy array describing the point cloud containing that component type. E.g.
```
clouds = {
    "curved_walls": model.gen_curved_walls(DENSITY, NOISE_STD),
    "planes": model.gen_planes(DENSITY, NOISE_STD),
    "tbeams": model.gen_tbeams_many(DENSITY, NOISE_STD),
}
```

Pass the cloud of the entire scene `clouds` into a Cloud object along with the location of the cutting plane along the z axis and the tolerance for deviation from the cutting plane
```
my_cloud = path.Cloud(cloud, z-plane=4.9, tolerance=1.1)
```
 
For each component group, create a ComponentGroup object with the n x 3 point cloud containing all components of that type and combine those into a list
```
component_groups = []
for key, value in clouds.items():
    component_groups.append(
        contour.ComponentGroup(
            key,
            value,
            Z_PLANE,
            GRID_DENSITY,
            TOLERANCE,
            my_cloud.grid_dim,
        )
    )
```

Create a path object with the list of component groups
```
my_path = path.Path(component_groups, GRID_DENSITY, Z_PLANE)
```

At this point, the Path object will create Component objects for each identified component, stitch them together, create an ordered list of points that describes the cutting path, then bring them back to point cloud space.
```
print(Complete cutting path is \n",my_path.coords3d) # Ordered list of points for the robot to follow
```

View information about the result for debugging and/or fun 
```
for component in my_path.components:
  print("Path for ",component.name," is \n",component.cntr)
print("2D cutting path in grid space is \n" my_path.coords2d")
```
## Extending the Code
### Adding Primitives 
To add a new primitive, navigate to contour.py -> Class ComponentGroup -> get_contours(). Add a new statement 
```if self.name == "my_new_component":```
Within this if statement, analyze the 2 x n numpy array `grid` and return a list of lists, where each list is an ordered list of key points that the torch will move between to cut the specific component.

**Recommendation:**
When an unknown component name is found for self.name, the program falls back to using cv2.findContours(). Visualize the performance with my_path_object.visualize(). If the output is close to acceptable, use the output as a base and modify it. If the performance is poor, consider applying Hough transforms and analyzing the intersections. 

The T-beam logic, for example, applies the cv2.findContours function, removes duplicate values, and adjusts the location of the end points

**Note:**
ComponentGroup expects all members of a component group to be passed together in a single binary image, in accordance with the problem definition. Code for new primitives must be able to identify the individual components and separate them into individual lists.these

**Note:**
The value of `self.name` will be taken from the dictionary used to create the ComponentGroup objects.

### Extending Synthetic Data Generation
Point clouds representing new structure types can be added to gen.py. This section is function-oriented. Create a new function named my_structure() that takes origin (can also be conceptualized as translation), density in dots per cubic unit, roll/pitch/yaw and dimension lengths. Consider using the plane and curved_wall functions to simplify the construction. Return the point cloud as an n x 3 numpy array 

## Adding Primitives 
To add a new primitive, navigate to contour.py -> Class ComponentGroup -> get_contours(). Add a new statement 
```if self.name = my_new_component:```
Within this if statement, analyze the 2 x n numpy array `grid` and return a list of lists, where each list is an ordered list of key points that the torch will move between to cut the specific component.

**Recommendation:**
When an unknown component name is found for self.name, the program falls back to using cv2.findContours(). Visualize the performance with my_path_object.visualize(). If the output is close to acceptable, use the output as a base and modify it. If the performance is poor, consider applying Hough transforms and analyzing the intersections. 

The T-beam logic, for example, apples the cv2.findContours function, removes duplicate values, and adjusts the location of the end points

**Note:**
ComponentGroup expects all members of a component group to be passed together in a single binary image, in accordance with the problem definition. Code for new primitives must be able to identify the individual components and separate them into individual lists.these

**Note:**
The value of `self.name` will be taken from the dictionary used to create the ComponentGroup objects.


## How It Works
- Cloud object holding the tagged point cloud of the vessel interior is created. Each point cloud representing a component type is stored as a separate dictionary entry. E.g.
    planes: [array of points labeled as plane] 
    bulkheads: [array of points labeled as bulkhead]
  - object calculates the relationship between cloud space and grid space
- ComponentGroup object for each type of component is created
  - object is initialized with the cloud containing all components of a given type, cutting plane location, desired grid density, desired tolerance, and dimensions and size/location of grid from Cloud object 
  - ep.extract_plane() is called to cut the components and save the cross-section as self.grid
  - grid is analyzed and a list of lists containing the ordered key points of each component is returned to self.cntrs
- List of ComponentGroup objects is fed into a Path object
- Path object creates a list of Component objects each containing a path for a specific ship component 
  - Each Component has a self.cntr property which specifies the path the robot follows to cut that component and a unique name to aid analysis
  - The cluster / brother sets are created using the forward and backward version of the component primtive paths 
  - Path's stitch_primitives() method returns an ordered list of component objects using the bound and branch algorithm to Path.
  - The cutting paths from the ordered list of components are concatenated to form a complete path in grid space as Path.coords2d 
  - The grid space coordinates are transformed into cloud space, giving the final ordered list of points for the robot

## Glossary
component:        specific I-beam, T-beam, bulb flat, or other part of ship 
component group:  all components of the same type, i.e. all bulb flats

