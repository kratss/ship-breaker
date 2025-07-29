Program flow:
- Created a Cloud object holding the tagged point cloud of the vessel interior. Each point cloud representing a component type is stored as a separate dictionary entry. i.g.
    planes: [array of points labeled as plane] 
    bulkheads: [array of points labeled as bulkhead]

- Create a ComponentGroup object for each type of component
- Feed list of ComponentGroup objects into a Path object
- Path object creates a list of Component objects each containing a path for a specific ship component 
  - Each Component has a .cntr property which specifies the path the robot follows for that component 
- Using these Component.cntr paths, Path gets an order of components from stitch_primitives
- Path then creates a full list of 3d coordinates that the robot must follow

# Path Planning for Ship Breaking 
This program takes a segmented point cloud and desired cutting plane, and returns an ordered list of points that create a cutting path for the cutting robot to follow with a plasma torch.

## Usage 
For each type of structure (component) labeled in the point cloud, create a ComponentGroup object. 
Example:
`from contour import ComponentGroup
from contour import Component 
from path import Path
from path import Cloud`

Pass the cloud of the entire scene (n x 3 numpy array) into a Cloud object with the location of the cutting plane along the z axis and the tolerance for deviation from the cutting plane
`my_cloud = path.Cloud(cloud, z-plane=3, tolerance=1)`

For each component group, create a ComponentGroup object with the n x 3 point cloud containing all components of that type
`tbeams = ComponentGroup("tbeams", tbeams_point_cloud, cutting_plane=3, tolerance=1, my_cloud.grid_dim)`


my_cloud = Cloud(
myTbeams = ComponentGroup(tbeam,[3 x n cloud containing T-beams],z_plane=3,grid_density=10,tolernace=0.5,`
## Adding Primitives 
To add a new primitive, navigate to contour.py -> Class ComponentGroup -> get_contours(). Add a new statement `if self.name = my_new_component:`. Analyze the 2 x n numpy array `grid` and return a list of lists, where each list is an ordered list of key points that the torch will be moved between to cut primitive. 

NOTE: ComponentGroup expects all members of a component group to be passed together in a single binary image, in accordance with the problem definition. Code for new primitives must be able to identify the individual components and separate them into individual lists.
## Glossary
component:        specific I-beam, T-beam, bulb flat, or other part of ship 
component group:  all components of the same type, i.e. all bulb flats

