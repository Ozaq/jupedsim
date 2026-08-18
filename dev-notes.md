# Dev Notes

## Task => Fix geometry lookup in 3D

**Issues:**

* Create connectivity graph for regions
* 

I think I will remove 
- WallMerge
- PolylineMerge
- WallRange

We need to discuss inputs, what are users supplying? When we expect them to
pass us a mesh we need to accept the fact that this mesh may be triagnulated
differently than delauny. Which is what we want anyways.

## Routing with meshes

We need to use
(isotropic_remeshing)[https://doc.cgal.org/6.1/Polygon_mesh_processing/group__PMP__meshing__grp.html#ga66cb01cf228ed22f0a2a4]74cfa2aeb3f]


## Crituque

- Routing does not work on its own mesh, all functionality is tied to geometry 3D. I **specifically** asked for this seperation.



