import bpy
import json

# sample data

f = open('/home/totoro/CG/ReproduceThem/ReconstructThin/Testing/curves.json', 'r')
text = f.read()
f.close()

curves = json.loads(text)["curves"]

cnt = 0

for curve in curves:
    # create the Curve Datablock
    curveData = bpy.data.curves.new('myCurve.' + str(cnt), type='CURVE')
    curveData.dimensions = '3D'
    curveData.resolution_u = 2

    # map coords to spline
    polyline = curveData.splines.new('NURBS')
    polyline.points.add(len(curve))
    for i, coord in enumerate(curve):
        x,y,z = coord
        polyline.points[i].co = (x, y, z, 1)

    # create Object
    curveOB = bpy.data.objects.new('myCurve.' + str(cnt), curveData)

    # attach to scene and validate context
    scn = bpy.context.scene
    scn.objects.link(curveOB)
    scn.objects.active = curveOB
    curveOB.select = True
