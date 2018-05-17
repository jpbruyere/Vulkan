import bpy

ob = bpy.ops.object
o = bpy.context.object
obj = bpy.context.active_object

if o.mode == 'EDIT':
    ob.editmode_toggle()

for polygon in obj.data.polygons:
    polygon.select = False

polys = len(obj.data.polygons)

for x in range(0, polys-1):
    obj.data.polygons[0].select = True
    ob.editmode_toggle()
    bpy.ops.mesh.separate(type='SELECTED')
    ob.editmode_toggle()
    



