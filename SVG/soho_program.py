import hou
import soho, sohog

import xml.etree.ElementTree as ET

class Poly(object):
    def __init__(self, geo, prim):
        self.geo = geo
        self.prim = prim
        self.points = []
        
        self.soppath = geo.globalValue('geo:soppath')[0]
        
        closed = geo.attribute('geo:prim', 'intrinsic:closed')
        self.closed = bool(geo.value(closed, self.prim)[0])

        vtx_count = geo.attribute('geo:prim', 'geo:vertexcount')
        pointref = geo.attribute('geo:vertex', 'geo:pointref')
        
        for vtx in range(geo.value(vtx_count, self.prim)[0]):
            i = geo.vertex(pointref, self.prim, vtx)[0]
            self.points.append(i)
            
        pointP = geo.attribute('geo:point',  'P')    
        self.P = [hou.Vector3(geo.value(pointP,i)) for i in self.points]
        
        self.projectedP = []
            
    def getPointsAttrib(self, attrib):
        attrib = geo.attribute('geo:point',  attrib)
        return [self.geo.value(attrib,i) for i in self.points]
        
    def getVerticesAttrib(self, attrib):
        attrib = geo.attribute('geo:vertex',  attrib)
        vtx_count = geo.attribute('geo:prim', 'geo:vertexcount')
        return [self.geo.vertex(attrib, self.prim, i) for i in range(self.geo.value(vtx_count, self.prim)[0])]
        
    def getPrimAttrib(self, attrib):
        attrib = geo.attribute('geo:prim',  attrib)
        if attrib < 0 :
            return
        size = self.geo.attribProperty(attrib, 'geo:vectorsize')
        if not size :
            return
        size = size[0]
        val = self.geo.value(attrib, self.prim)
        if val and len(val)!=size:
            return
        if val and size == 1 :
            return val[0]
        return val
        
    def project(self, cam_matrix, projection_matrix):
        self.projectedP = [toNDC(P,cam_matrix, projection_matrix) for P in self.P]
        return
        
    def projectedDepth(self, method = "min"):
        if method == "avg" or method == "average" :
            return sum([P.z() for P in self.projectedP])/len(self.points)
        elif method == "max" or method == "maximum" :
            return max([P.z() for P in self.projectedP])
        elif method == "min" or method == "minumum" :
            return min([P.z() for P in self.projectedP])
            
    def SVGProjected(self, resolution):
        if not self.projectedP :
            return
        data = "M{0} {1} ".format(self.projectedP[0].x() * resolution[0], (1-self.projectedP[0].y()) * resolution[1])
        for P in self.projectedP[1:]:
            data += "L{0} {1}".format(P.x() * resolution[0], (1-P.y()) * resolution[1])
        if self.closed :
            data += 'Z'
        return data
        
    def SVGAttribute(self, attribute = 'uv', resolution = [1024,1024], scale = True):
        point_attrs = self.geo.globalValue('geo:pointattribs')
        vtx_attrs = self.geo.globalValue('geo:vertexattribs')
        if attribute in vtx_attrs:
            positions = self.getVerticesAttrib(attribute)
        elif attribute in point_attrs:
            positions = self.getPointsAttrib(attribute)
        else:
            return
        scaled = []
        if scale :
            for pos in positions :
                s = [pos[0] * resolution[0], (1-pos[1]) * resolution[1]]
                scaled.append(s)
            positions = scaled
        data = "M{0} {1} ".format(positions[0][0], positions[0][1])
        for P in positions[1:]:
            data += "L{0} {1}".format(P[0], P[1])
        if self.closed :
            data += 'Z'
        return data
            
def toNDC(P, cam_matrix, projection_matrix):
    P = hou.Vector4(P)
    P[3] = 1.0
    P *= cam_matrix.inverted()
    P *= projection_matrix
    P[0] = (P[0]/(P[3]*2)) + .5
    P[1] = (P[1]/(P[3]*2)) + .5
    P[2] /= P[3]
    P[3] = 1.0
    P = hou.Vector3(P)
    return P

        
node = hou.pwd()
module = node.hm()

controlParameters = {
    'now'     : soho.SohoParm('state:time',  'real', [0], False,  key='now'),
    'fps'     : soho.SohoParm('state:fps',   'real', [24], False, key='fps'),
    'camera'  : soho.SohoParm('camera', 'string', ['/obj/cam1'], False),
    'vm_picture'    : soho.SohoParm('vm_picture', 'string', ['$HIP/render/$HIPNAME.$OS.$F4.svg'], False),
    'render_viewcamera' :soho.SohoParm('render_viewcamera', 'bool', [True], False),
    'projection_attribute' :soho.SohoParm('projection_attribute', 'string', ['uv'], False),
    'attribute_rendering_resolution' :soho.SohoParm('attribute_rendering_resolution', 'int', [1024,1024], False),
    'attribute_scale_by_resolution' :soho.SohoParm('attribute_scale_by_resolution', 'bool', [True], False)
    
}

parmlist = soho.evaluate(controlParameters)

project = bool(parmlist['render_viewcamera'].Value[0])
projection_attribute = parmlist['projection_attribute'].Value[0]
attribute_rendering_resolution = [parmlist['attribute_rendering_resolution'].Value[0],parmlist['attribute_rendering_resolution'].Value[1]]
attribute_scale_by_resolution = bool(parmlist['attribute_scale_by_resolution'].Value[0])


now     = parmlist['now'].Value[0]
camera  = parmlist['camera'].Value[0]
fps     = parmlist['fps'].Value[0]
options = {"state:autoheadlight" : False}

if not soho.initialize(now = now, camera = camera, options = options):
    soho.error('Unable to initialize rendering module with camera: {0}'.format(repr(camera)))
    
for cam in soho.objectList('objlist:camera'):
    break
else:
    soho.error("Unable to find viewing camera for render")
    
camParms = {
    'space:world':soho.SohoParm('space:world',       'real', [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], False),
    'focal'      :soho.SohoParm('focal',             'real', [0.050], False),
    'aperture'   :soho.SohoParm('aperture',          'real', [0.0414214], False),
    'orthowidth' :soho.SohoParm('orthowidth',        'real', [2], False),
    'near'       :soho.SohoParm('near',              'real', [0.001], False),
    'far'        :soho.SohoParm('far',               'real', [1000], False),
    'res'        :soho.SohoParm('res',               'int', [1920,1080], False),
    'aspect'     :soho.SohoParm('aspect',            'real', [1], False),
    'win'        :soho.SohoParm('win',               'real', [0,0], False),
    'winsize'    :soho.SohoParm('winsize',           'real', [1,1], False),
    'projection' :soho.SohoParm('projection',        'string', [0,0], False),
}

cam.evaluate(camParms, now)

resolution = [float(camParms['res'].Value[0]),float(camParms['res'].Value[1])]
if not project :
    resolution = attribute_rendering_resolution

cam_matrix = hou.Matrix4( camParms['space:world'].Value)
projection_matrix = hou.Matrix4()
projection_matrix.setToIdentity()

if camParms['projection'].Value[0] == "perspective" :
    zoom         = camParms['focal'].Value[0] / camParms['aperture'].Value[0]
    image_aspect = resolution[0] / resolution[1]
    pixel_aspect = camParms['aspect'].Value[0]
    clip_near    = camParms['near'].Value[0]
    clip_far     = camParms['far'].Value[0]
    window_xmin  = camParms['win'].Value[0]
    window_xmax  = camParms['winsize'].Value[0]
    window_ymin  = camParms['win'].Value[1]
    window_ymax  = camParms['winsize'].Value[1]
    projection_matrix.setToPerspective(zoom,
                                       image_aspect,
                                       pixel_aspect,
                                       clip_near,
                                       clip_far,
                                       window_xmin,
                                       window_xmax,
                                       window_ymin,
                                       window_ymax)
elif camParms['projection'].Value[0] == "ortho" :
    zoom         = camParms['focal'].Value[0] / camParms['aperture'].Value[0]
    orthowidth   = camParms['orthowidth'].Value[0]
    image_aspect = resolution[0] /resolution[1]
    pixel_aspect = camParms['aspect'].Value[0]
    clip_near    = camParms['near'].Value[0]
    clip_far     = camParms['far'].Value[0]
    window_xmin  = camParms['win'].Value[0]
    window_xmax  = camParms['winsize'].Value[0]
    window_ymin  = camParms['win'].Value[1]
    window_ymax  = camParms['winsize'].Value[1]
    projection_matrix.setToOrthographic(zoom,
                                        image_aspect,
                                        pixel_aspect,
                                        clip_near,
                                        clip_far,
                                        window_xmin,
                                        window_xmax,
                                        window_ymin,
                                        window_ymax)
else :
    soho.error("Unrecognized camera projection type")

objectSelection = {
    'vobject'       : soho.SohoParm('vobject', 'string',       ['*'], False),
    'forceobject'   : soho.SohoParm('forceobject',     'string',       [''], False),
    'excludeobject' : soho.SohoParm('excludeobject', 'string',       [''], False),
}    
    
objparms = soho.evaluate(objectSelection, now)

stdobject = objparms['vobject'].Value[0]
forceobject = objparms['forceobject'].Value[0]
excludeobject = objparms['excludeobject'].Value[0]

    
soho.addObjects(now = now, geometry = stdobject, lights = "", fog = "", do_culling = True, geo_parm='vobject')
soho.addObjects(now = now, geometry = forceobject, lights = "", fog = "", do_culling = False, geo_parm='forceobject')
soho.removeObjects(now = now, geometry = excludeobject, lights = "*", fog = "*", geo_parm='excludeobject', light_parm='excludelights', fog_parm='excludefog')

attributeControlParms = {
    'z_sorting_method'       : soho.SohoParm('z_sorting_method',       'string', ['avg'],    False),
    'fill_color_attribute'   : soho.SohoParm('fill_color_attribute',   'string', ['Cd'],     False),
    'fill_alpha_attribute'   : soho.SohoParm('fill_alpha_attribute',   'string', ['Alpha'],  False),
    'stroke_color_attribute' : soho.SohoParm('stroke_color_attribute', 'string', ['Cd'],     False),
    'stroke_alpha_attribute' : soho.SohoParm('stroke_alpha_attribute', 'string', ['Alpha'],  False),
    'stroke_width_attribute' : soho.SohoParm('stroke_width_attribute', 'string', ['width'],  False),
    'force_stroke_attribute' : soho.SohoParm('force_stroke_attribute', 'string', ['stroke'], False),
    'stroke_linecap_attribute' : soho.SohoParm('stroke_linecap_attribute', 'string', ['linecap'], False),
    'stroke_linejoin_attribute' : soho.SohoParm('stroke_linejoin_attribute', 'string', ['linejoin'], False)
} 

attributeControlParms = soho.evaluate(attributeControlParms, now)
z_sorting_method = attributeControlParms['z_sorting_method'].Value[0]
fill_color_attribute = attributeControlParms['fill_color_attribute'].Value[0]
fill_alpha_attribute = attributeControlParms['fill_alpha_attribute'].Value[0]
stroke_color_attribute = attributeControlParms['stroke_color_attribute'].Value[0]
stroke_alpha_attribute = attributeControlParms['stroke_alpha_attribute'].Value[0]
stroke_width_attribute = attributeControlParms['stroke_width_attribute'].Value[0]
force_stroke_attribute = attributeControlParms['force_stroke_attribute'].Value[0]
stroke_linecap_attribute = attributeControlParms['stroke_linecap_attribute'].Value[0]
stroke_linejoin_attribute = attributeControlParms['stroke_linejoin_attribute'].Value[0]

soho.lockObjects(now)

prims = []

for obj in  soho.objectList('objlist:instance'):
    soppath = []
    if not obj.evalString('object:soppath', now, soppath):
        continue
    geo = sohog.SohoGeometry(soppath[0], now)
    numprims = geo.globalValue('geo:primcount')[0]
    for i, prim in enumerate(range(numprims)):
        poly = Poly(geo, prim)
        poly.project(cam_matrix,projection_matrix)
        prims.append(poly)
        
prims = sorted(prims, key = lambda prim : prim.projectedDepth(method = z_sorting_method), reverse = True)

svg = ET.Element('svg')
svg.set('width','{0}px'.format(resolution[0]))
svg.set('height','{0}px'.format(resolution[1]))

svg.set('xmlns','http://www.w3.org/2000/svg')

for prim in prims :
    path = ET.SubElement(svg,'path')
    path.set('id', "{0}#{1}".format(prim.soppath,prim.prim))
    data = ''
    if project :
        data = prim.SVGProjected(resolution)
    else :
        data = prim.SVGAttribute(projection_attribute, attribute_rendering_resolution, attribute_scale_by_resolution)
    path.set('d', data)
    fill_color = prim.getPrimAttrib(fill_color_attribute)
    stroke_color = prim.getPrimAttrib(stroke_color_attribute)
    stroke = bool(prim.getPrimAttrib(force_stroke_attribute))
    stroke_width = prim.getPrimAttrib(stroke_width_attribute) or 1.0
    if prim.closed and fill_color:
        path.set('fill','rgb({0},{1},{2})'.format(fill_color[0]*255,fill_color[1]*255,fill_color[2]*255))
        fill_alpha = prim.getPrimAttrib(fill_alpha_attribute)
        if fill_alpha :
            path.set('fill-opacity','{0}'.format(fill_alpha))
    if not prim.closed or not fill_color:
        path.set('fill','none')
    if (not prim.closed or stroke) and stroke_color and isinstance(stroke_color,list):
        path.set('stroke','rgb({0},{1},{2})'.format(stroke_color[0]*255,stroke_color[1]*255,stroke_color[2]*255))
        stroke_alpha = prim.getPrimAttrib(stroke_alpha_attribute) or 1.0
        path.set('stroke-width','{0}'.format(stroke_width))
        if stroke_alpha :
            path.set('stroke-opacity', '{0}'.format(stroke_alpha))
        linecap = prim.getPrimAttrib(stroke_linecap_attribute)
        if linecap and linecap in ['butt','round', 'square']:
            path.set('stroke-linecap', linecap)
        linejoin = prim.getPrimAttrib(stroke_linejoin_attribute)
        if linejoin and linejoin in ['arcs', 'bevel', 'miter', 'miter-clip', 'round']:
            path.set('stroke-linejoin', linejoin)
            
    

file = ET.ElementTree(element = svg)
file.write(parmlist['vm_picture'].Value[0])
soho.message('{0}'.format(parmlist['vm_picture'].Value[0]))