# 3D pointcloud meshing
Using Python + Open3D library - http://www.open3d.org/ 

## Usage
1. clone this repo

2. ``` conda env create -f environment.yml ```
3. ``` conda activate 3D ```
4. ``` python pcd2mesh.pt ```


## Parameters

VOX_DOWNSAMPLE: [float or None] if float the input pointcloud is cluster into voxels of give size  
NORMAL_EST: [Bool]  normal estimation step  
POISSON_DEPTH: [int]  poisson reconstruction octree depth, the higher, the more detail  
POISSON_DENSE_THRESH: [float] threshold for culling (cleaning) the meshed result  
FILTERING: [None, average, laplace, taubin]  
VERTEX_CLUSTERING: [int, None] #Mesh simplification resolution  
MESH_DECIMATIONL: [None, int] None or target number of triangles, only used if no vertex clustering  
KEEP_RAW_MESH: [bool] keep unfiltered and unsimplified mesh  

VISUAL_DEBUG: [bool] display all visalizations  
