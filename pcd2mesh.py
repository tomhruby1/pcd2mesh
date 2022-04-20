#!/usr/bin/env python

# TODO: args 

import sys
import open3d as o3d 
import numpy as np
import matplotlib.pyplot as plt

#params
INPUT_PATH = "laser/ext_court.ply"
OUTPUT_PATH = "export/court_mesh_simp512"

VOX_DOWNSAMPLE = None #float or None, depends on the model
NORMAL_EST = True #normal estimation 
POISSON_DEPTH = 11  #poisson reconstruction depth, the higher, the more detail
POISSON_DENSE_THRESH = 0.2
FILTERING = None #or average, laplace, taubin
VERTEX_CLUSTERING = 256 #int or None
MESH_DECIMATION = None #None or target number of triangles, only if no clustering
KEEP_RAW_MESH = False #keep unfiltered and undecimated mesh

VISUAL_DEBUG = False


def voxel_downsample(pcd):
    ''' voxel downsample '''
    downpcd = pcd.voxel_down_sample(voxel_size=VOX_DOWNSAMPLE)
    print(downpcd)
    o3d.visualization.draw_geometries([downpcd])
    return downpcd

def poisson_reconstruction(pcd):
    '''poisson s. reconstruction'''

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                            pcd, depth=POISSON_DEPTH, linear_fit=False)
    print(mesh)
    if VISUAL_DEBUG: o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    #Ad poisson s. recon. --Remove low density vertices
    #mesh density visualization more yellow more dense
    #visualize densities
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    if VISUAL_DEBUG: o3d.visualization.draw_geometries([density_mesh])

    print('Removing low density vertices...')
    vertices_to_remove = densities < np.quantile(densities, POISSON_DENSE_THRESH)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    print(mesh)
    if VISUAL_DEBUG: o3d.visualization.draw_geometries([mesh],  mesh_show_back_face=True)

    return mesh

def vertex_clustering(mesh, precision):
    '''vertex downsample via voxelization based on VOXEL size'''
    print("Vertex clustering...")
    mesh_in = mesh
    voxel_size = max(mesh_in.get_max_bound() - mesh_in.get_min_bound()) / precision
    print(f'voxel_size = {voxel_size:e}')
    mesh_smp = mesh_in.simplify_vertex_clustering(
    voxel_size=voxel_size,
    contraction=o3d.geometry.SimplificationContraction.Average)
    print(
        f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles'
    )
    o3d.visualization.draw_geometries([mesh_smp], mesh_show_wireframe=True)

    return mesh_smp


if __name__=="__main__":
    #print("arg1:", sys.argv[1])
    
    pcd = o3d.io.read_point_cloud(INPUT_PATH)
    print("Loaded", pcd)
    o3d.visualization.draw_geometries([pcd])
    
    #pointcloud preprocess    
    if VOX_DOWNSAMPLE:
        print("Voxel downsampling...")
        pcd = voxel_downsample(pcd)

    if NORMAL_EST:
        print("Estimating normals...")
        pcd.estimate_normals()
        if VISUAL_DEBUG: o3d.visualization.draw_geometries([pcd], point_show_normal=True)
    
    
    print("\n--- RECONSTRUCTION ---")
    mesh = poisson_reconstruction(pcd)
    
    print("\n--- FILTERING ---")
    mesh_filt = mesh
    if FILTERING == 'average':
        mesh_filt = mesh.filter_smooth_simple(number_of_iterations=10)
        mesh_filt.compute_vertex_normals()
        if VISUAL_DEBUG: o3d.visualization.draw_geometries([mesh_filt])
    
    elif FILTERING == 'laplace':
        #laplacian filter
        mesh_filt = mesh.filter_smooth_laplacian(number_of_iterations=5)
        mesh_filt.compute_vertex_normals()
        if VISUAL_DEBUG: o3d.visualization.draw_geometries([mesh_filt])

    elif FILTERING == 'taubin':
        #taubin filter
        mesh_filt = mesh.filter_smooth_taubin(number_of_iterations=10)
        mesh_filt.compute_vertex_normals()
        if VISUAL_DEBUG: o3d.visualization.draw_geometries([mesh_filt], mesh_show_wireframe=False)

    #mesh simplification
    #o3d.visualization.draw_geometries([mesh_filt], mesh_show_wireframe=False, mesh_show_back_face=True)
    print(
        f'Raw mesh has {len(mesh_filt.vertices)} vertices and {len(mesh_filt.triangles)} triangles')
    
    if VERTEX_CLUSTERING:
        mesh_smp = vertex_clustering(mesh, VERTEX_CLUSTERING)

    elif MESH_DECIMATION:
        print("\n--- MESH DECIMATION ---")
        mesh_smp = mesh.simplify_quadric_decimation(target_number_of_triangles=65000)
        print(
            f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles'
        )
        #o3d.visualization.draw_geometries([mesh_smp], mesh_show_wireframe=True)
        if VISUAL_DEBUG: o3d.visualization.draw_geometries([mesh_smp], mesh_show_wireframe=False, 
                                                            mesh_show_back_face=True)
    else: mesh_smp = mesh

    o3d.visualization.draw_geometries([mesh_smp], mesh_show_wireframe=False, mesh_show_back_face=True)
    #export
    print("\n--- EXPORTING ---")
    o3d.io.write_triangle_mesh(OUTPUT_PATH+".obj",
                               mesh_smp,
                               write_triangle_uvs=True,
                               print_progress=True)
    if KEEP_RAW_MESH:
        print("exporting raw mesh...")
        o3d.io.write_triangle_mesh(OUTPUT_PATH+"_raw.obj",
                                    mesh,
                                    write_triangle_uvs=True,
                                    print_progress=True)