Program flow:
- Get a tagged point cloud of the vessel interior
- Create a ComponentGroup object for each type of component
- Feed list of ComponentGroup objects into a Path object
- Path object creates a list of Component objects each containing a path for a specific ship component 
  - Each Component has a .cntr property which specifies the path the robot follows for that component 
- Using these Component.cntr paths, Path gets an order of components from stitch_primitives
- Path then creates a full list of 3d coordinates that the robot must follow

## Glossary
component:    refers to a specific I-beam, T-beam, bulb flat, or other part of ship 

