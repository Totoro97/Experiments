import bpy
import math, os

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

def GenerateRenderResult() :
    for ob in bpy.context.scene.objects:
        if ob.type == 'CAMERA':
            bpy.context.scene.camera = ob
            print('Set camera %s' % ob.name )
            file = os.path.join("C:/tmp", ob.name )
            bpy.context.scene.render.filepath = file
            bpy.ops.render.render( write_still=True )
