from bpy import context, data, ops
import random, os
import math
def RandFloat(L, R) :
    return L + random.random() * (R - L)
    
def GenerateLights() :
    n = random.randint(1, 3)
    for _ in range(n) :
        while True :
            position = (RandFloat(-6.0, 6.0), RandFloat(-6.0, 6.0), RandFloat(-6.0, 6.0))
            length = 0
            for p in position :
                length += p * p
            if length > 16.0 :
                break
        ops.object.lamp_add(
            type = 'POINT',
            view_align = False,
            location = position
        )
        
def GenerateRandomCurve() :
    # Create a bezier circle and enter edit mode.
    if random.randint(0, 1) == 1 :
        ops.curve.primitive_bezier_curve_add(radius=random.random(),
                                             location=(0.0, 0.0, 0.0),
                                             enter_editmode=True)
    else :
        ops.curve.primitive_bezier_circle_add(radius=random.random(),
                                             location=(0.0, 0.0, 0.0),
                                             enter_editmode=True)

    # Subdivide the curve by a number of cuts, giving the
    # random vertex function more points to work with.
    ops.curve.subdivide(number_cuts=random.randint(1, 8))

    # Randomize the vertices of the bezier circle.
    # offset [-inf .. inf], uniform [0.0 .. 1.0],
    # normal [0.0 .. 1.0], RNG seed [0 .. 10000].
    ops.transform.vertex_random(offset=1.0, uniform=0.1, normal=0.0, seed=0)

    # Scale the curve while in edit mode.
    ops.transform.resize(
        value = (RandFloat(0.5, 3.0), RandFloat(0.5, 3.0), RandFloat(0.5, 3.0))
    )
    ops.transform.translate(
        value = (RandFloat(-4.0, 4.0), RandFloat(-4.0, 4.0), RandFloat(-4.0, 4.0)))
    ops.transform.rotate(
        value = RandFloat(0, math.pi),
        axis = (RandFloat(-1.0, 1.0), RandFloat(-1.0, 1.0), RandFloat(-1.0, 1.0))
    )
    # Return to object mode.
    ops.object.mode_set(mode='OBJECT')

    # Store a shortcut to the curve object's data.
    obj_data = context.active_object.data

    # Which parts of the curve to extrude ['HALF', 'FRONT', 'BACK', 'FULL'].
    obj_data.fill_mode = 'FULL'

    # Breadth of extrusion.
    obj_data.extrude = 0

    # Smoothness of the segments on the curve.
    obj_data.resolution_u = 20
    obj_data.render_resolution_u = 32

    ops.curve.primitive_bezier_circle_add(radius=random.random() * 0.03 + 0.01, enter_editmode=True)
    ops.curve.subdivide(number_cuts=4)
    bevel_control = context.active_object
    bevel_control.data.name = bevel_control.name = 'Bevel Control'

    obj_data.bevel_object = bevel_control
    ops.object.mode_set(mode='OBJECT')
    
    mat = obj_data.materials.get("Material")
    if mat == None:
        # create material
        mat = data.materials.new(name="Material")

    # Assign it to object
    if obj_data.materials:
        # assign to 1st material slot
        obj_data.materials[0] = mat
    else:
        # no slots
        obj_data.materials.append(mat)
    mat.diffuse_color = (random.random(), random.random(), random.random())

def ClearAllObjects() :
    ops.object.select_all(action='DESELECT')
    for obj in data.objects :
        if obj.name == 'Camera' :
            continue
        obj.select = True
    ops.object.delete()

def GenerateRenderResult(img_name = 'tmp') :
    context.scene.render.engine = 'CYCLES'
    context.scene.cycles.film_transparent = True
    for obj in data.objects:
        if obj.type == 'CAMERA':
            context.scene.camera = obj
            cam_name = obj.name
            cam_id = 0
            print('Set camera %s' % obj.name)
            file_name = '/home/totoro/tmp/' + img_name
            context.scene.render.filepath = file_name
            ops.render.render(write_still=True)

def GenerateManyCurveImages(num_img) :
    l = len(str(num_img))
    for _ in range(num_img) :
        ClearAllObjects()
        GenerateRandomCurve()
        GenerateLights()
        GenerateRenderResult(('%.' + str(l) + 'd') % _)

def 
GenerateManyCurveImages(500)
