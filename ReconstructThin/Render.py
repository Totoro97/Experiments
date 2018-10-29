import bpy
import math, os
from mathutils import Matrix, Vector

def GenerateCameras(num_camera) :
    for i in range(num_camera) :
        ang = 2.0 * math.pi / num_camera * i
        x = 7 * math.cos(ang)
        y = 7 * math.sin(ang)
        bpy.ops.object.camera_add(view_align=True, enter_editmode=False, location=(x, y, 0))

    for ob in bpy.context.scene.objects :
        if ob.type != 'CAMERA' :
            continue
        bpy.context.scene.objects.active = ob
        bpy.ops.object.constraint_add(type='TRACK_TO')
        bpy.context.object.constraints["Track To"].target = bpy.data.objects["NurbsPath"]
        bpy.context.object.constraints["Track To"].track_axis = 'TRACK_NEGATIVE_Z'
        bpy.context.object.constraints["Track To"].up_axis = 'UP_Y'

def GenerateRenderResult(num_cam) :
    for ob in bpy.context.scene.objects:
        if ob.type == 'CAMERA':
            bpy.context.scene.camera = ob
            cam_name = ob.name
            cam_id = 0
            for i in range(num_cam) :
                if str(i) in cam_name :
                    cam_id = i
            print('Set camera %s' % ob.name)
            file = os.path.join(
                "C:/Users/Aska/Project/ReproduceThem/ReconstructThin/data", str(cam_id))
            bpy.context.scene.render.filepath = file
            bpy.ops.render.render( write_still=True )

#---------------------------------------------------------------------------------------------------
# 3x4 P matrix from Blender camera
# Reference: https://blender.stackexchange.com/questions/38009/3x4-camera-matrix-from-blender-camera
#---------------------------------------------------------------------------------------------------

# BKE_camera_sensor_size
def GetSenserSize(sensor_fit, sensor_x, sensor_y):
    if sensor_fit == 'VERTICAL':
        return sensor_y
    return sensor_x

# BKE_camera_sensor_fit
def GetSensorFit(sensor_fit, size_x, size_y):
    if sensor_fit == 'AUTO':
        if size_x >= size_y:
            return 'HORIZONTAL'
        else:
            return 'VERTICAL'
    return sensor_fit

# Build intrinsic camera parameters from Blender camera data
#
# See notes on this in 
# blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model
# as well as
# https://blender.stackexchange.com/a/120063/3581
def GetCalibrationMatrixKFromBlender(camd):
    if camd.type != 'PERSP':
        raise ValueError('Non-perspective cameras not supported')
    scene = bpy.context.scene
    f_in_mm = camd.lens
    scale = scene.render.resolution_percentage / 100
    resolution_x_in_px = scale * scene.render.resolution_x
    resolution_y_in_px = scale * scene.render.resolution_y
    sensor_size_in_mm = GetSenserSize(camd.sensor_fit, camd.sensor_width, camd.sensor_height)
    sensor_fit = GetSensorFit(
        camd.sensor_fit,
        scene.render.pixel_aspect_x * resolution_x_in_px,
        scene.render.pixel_aspect_y * resolution_y_in_px
    )
    pixel_aspect_ratio = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x
    if sensor_fit == 'HORIZONTAL':
        view_fac_in_px = resolution_x_in_px
    else:
        view_fac_in_px = pixel_aspect_ratio * resolution_y_in_px
    pixel_size_mm_per_px = sensor_size_in_mm / f_in_mm / view_fac_in_px
    s_u = 1 / pixel_size_mm_per_px
    s_v = 1 / pixel_size_mm_per_px / pixel_aspect_ratio

    # Parameters of intrinsic calibration matrix K
    u_0 = resolution_x_in_px / 2 - camd.shift_x * view_fac_in_px
    v_0 = resolution_y_in_px / 2 + camd.shift_y * view_fac_in_px / pixel_aspect_ratio
    skew = 0 # only use rectangular pixels

    K = Matrix(
        ((s_u, skew, u_0),
        (   0,  s_v, v_0),
        (   0,    0,   1)))
    return K

# Returns camera rotation and translation matrices from Blender.
# 
# There are 3 coordinate systems involved:
#    1. The World coordinates: "world"
#       - right-handed
#    2. The Blender camera coordinates: "bcam"
#       - x is horizontal
#       - y is up
#       - right-handed: negative z look-at direction
#    3. The desired computer vision camera coordinates: "cv"
#       - x is horizontal
#       - y is down (to align to the actual pixel coordinates 
#         used in digital images)
#       - right-handed: positive z look-at direction

def Get3x4RTMatrixFromBlender(cam):
    # bcam stands for blender camera
    R_bcam2cv = Matrix(
        ((1, 0,  0),
        (0, -1, 0),
        (0, 0, -1)))

    # Transpose since the rotation is object rotation, 
    # and we want coordinate rotation
    # R_world2bcam = cam.rotation_euler.to_matrix().transposed()
    # T_world2bcam = -1*R_world2bcam * location
    #
    # Use matrix_world instead to account for all constraints
    location, rotation = cam.matrix_world.decompose()[0:2]
    print('location = ' + str(location))
    R_world2bcam = rotation.to_matrix().transposed()

    # Convert camera location to translation vector used in coordinate changes
    # T_world2bcam = -1*R_world2bcam*cam.location
    # Use location from matrix_world to account for constraints:     
    T_world2bcam = -1*R_world2bcam * location
    print('T_world2bcam = ' + str(T_world2bcam))
    # Build the coordinate transform matrix from world to computer vision camera
    R_world2cv = R_bcam2cv*R_world2bcam
    T_world2cv = R_bcam2cv*T_world2bcam

    # put into 3x4 matrix
    RT = Matrix((
        R_world2cv[0][:] + (T_world2cv[0],),
        R_world2cv[1][:] + (T_world2cv[1],),
        R_world2cv[2][:] + (T_world2cv[2],)
        ))
    return R_world2cv, T_world2cv, RT

def Get3x4PMatrixFromBlender(cam):
    K = GetCalibrationMatrixKFromBlender(cam.data)
    R, T, RT = Get3x4RTMatrixFromBlender(cam)
    return K, R, T

def GetCameraMatrixFromBlender() :
    f = open('C:/Users/Aska/Project/ReproduceThem/ReconstructThin/data/Cameras.txt', 'w')
    text = ''
    for cam in bpy.context.scene.objects :
        if cam.type != 'CAMERA' :
            continue
        K, R, T = Get3x4PMatrixFromBlender(cam)
        for i in range(3) :
            for j in range(3) :
                text += str(K[i][j]) + ' '
            text += '\n'
        for i in range(3) :
            for j in range(3) :
                text += str(R[i][j]) + ' '
            text += '\n'
        for i in range(3) :
            text += str(T[i]) + '\n'
    f.write(text)
    f.close()

if __name__ == '__main__' :
    GenerateCameras(32)
    GenerateRenderResult(32)
    GetCameraMatrixFromBlender()