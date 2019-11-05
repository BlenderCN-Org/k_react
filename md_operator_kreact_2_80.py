# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

'''
TO DO LIST:

- hold bug
- complex search for closest_vertex
- pause (and pause on mousemove)
- snap preview based on mouse position
- nudge function

'''

bl_info = {
    "name": "K_react",
    "description": "Create quad/trig/line",
    "author": "Mihai Dobrin",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "View3D ",
    "warning": "", # used for warning icon and text in addons panel
    "wiki_url": "http://",
    "tracker_url": "http://"
                   "func=detail&aid=<number>",
    "category": "Tools"}

import os
import sys

import bpy
import bgl
import blf
from bpy_extras import view3d_utils
import bmesh
from math import pi, atan, acos, degrees, sqrt
from mathutils import kdtree, Vector, Matrix
from functools import reduce
from time import time

import linecache
import sys

# EDIT SETTINGS ################################################################
verbose = False

SNAP_RATIO       = 0.15
PIXEL_THRESHOLD  = 4
MAX_SEARCH       = 10
REFRESH_INTERVAL = 33
SNAP_TOLERANCE   = 0.01
SOLUTION_DELAY   = 250
################################################################################

MODE_FACE = 0
MODE_TRIG = 1

FACE_LINE = -2
FACE_NONE = -1
FACE_FREE =  0
FACE_SNAP =  1
FACE_FILL =  2
FACE_LINK =  3

addon_keymaps = []

current_time = lambda: int(round(time() * 1000))

def PrintException():
    exc_type, exc_obj, tb = sys.exc_info()
    f = tb.tb_frame
    lineno = tb.tb_lineno
    filename = f.f_code.co_filename
    linecache.checkcache(filename)
    line = linecache.getline(filename, lineno, f.f_globals)
    print( 'EXCEPTION IN ({}, LINE {} "{}"): {}'.format(filename, lineno, line.strip(), exc_obj))


def dist(a, b):
    return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def prod(iterable):
    return reduce(lambda x, y: x*y, iterable)


def get_closest_ray_hit_test(vertex, origin_obj, snap_obj, wire_normal = None):

    #print("Cv hit test:")
    #print("origin_obj ", origin_obj)
    #print("snap_obj ", snap_obj)
    #print("hit n ", wire_normal)

    current_obj_matrix = origin_obj.matrix_world
    current_obj_matrix_inv = current_obj_matrix.inverted()

    snap_obj_matrix = snap_obj.matrix_world
    snap_obj_matrix_inv = snap_obj_matrix.inverted()

    ray_origin = current_obj_matrix @ vertex.co
    ray_origin = snap_obj_matrix_inv @ ray_origin

    if(vertex.is_wire):
        ray_direction = wire_normal
    else:
        ray_direction = vertex.normal


    def obj_ray_cast(ray_nrm):
        ray_nrm = ray_nrm @ current_obj_matrix_inv  @ snap_obj_matrix
        success, location, normal, face_index = snap_obj.ray_cast(ray_origin, ray_nrm)

        if success:
            location = snap_obj_matrix @ location
            location = current_obj_matrix_inv @ location
            normal = normal @ snap_obj_matrix_inv @ current_obj_matrix

            return location, normal, face_index
        else:
            return None, None, None

    test_data = []

    front_hit, front_normal, front_face_index = obj_ray_cast(ray_direction)
    if front_hit is not None:
        front_hit_distance = dist(front_hit, ray_origin)
        test_data.append([front_hit_distance, front_normal])

    back_hit, back_normal, back_face_index = obj_ray_cast(-ray_direction)
    if back_hit is not None:
        back_hit_distance = dist(back_hit, ray_origin)
        test_data.append([back_hit_distance, back_normal])


    if(test_data != []):
        if(len(test_data) == 2):
            if(test_data[0][0] < test_data[1][0]):
                return test_data[0]
            else:
                return test_data[1]
        else:
            return test_data[0]
    else:
        return [float('inf'), None]

def get_ray_hit(context, x, y):
    current_obj = context.object
    current_obj_matrix = current_obj.matrix_world
    current_obj_matrix_inv = current_obj_matrix.inverted()
    scene = context.scene
    region = context.region
    rv3d = context.region_data
    coord = x, y

    view_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)

    ray_target = ray_origin + view_vector


    def visible_objects_and_duplis():
        """Loop over (object, matrix) pairs (mesh only)"""

        depsgraph = context.evaluated_depsgraph_get()
        for dup in depsgraph.object_instances:
            if dup.is_instance:  # Real dupli instance
                obj = dup.instance_object
                yield (obj)#, dup.matrix_world.copy())
            else:  # Usual object
                obj = dup.object
                yield (obj)#, obj.matrix_world.copy())

    def obj_ray_cast(obj):
        matrix = obj.matrix_world
        matrix_inv = matrix.inverted()
        ray_origin_obj = matrix_inv @ ray_origin
        ray_target_obj = matrix_inv @ ray_target
        ray_direction_obj = ray_target_obj - ray_origin_obj


        success, location, normal, face_index = obj.ray_cast(ray_origin_obj, ray_direction_obj)

        if success:
            return location, normal, face_index, matrix, matrix_inv
        else:
            return None, None, None, None, None

    best_length_squared = -1.0
    best_obj = None
    best_normal = None
    best_face_index = -1
    best_location   = None

    for obj in visible_objects_and_duplis():
        if obj.type == 'MESH' and obj.name != current_obj.name and len(obj.data.polygons) > 0 :
            hit, normal, face_index, matrix, matrix_inv = obj_ray_cast(obj)
            if hit is not None:
                hit_world = matrix @ hit
                length_squared = (hit_world - ray_origin).length_squared
                if best_obj is None or length_squared < best_length_squared:
                    best_length_squared = length_squared
                    best_obj = obj
                    best_normal = normal @ matrix_inv @ current_obj_matrix

                    best_location = hit
                    best_face_index = face_index

    if best_obj is not None:
        best_location = matrix @ best_location
        best_location = current_obj_matrix_inv @ best_location

        return  best_location, best_normal, best_obj, best_face_index
    else:
        return None, None, None, -1


class MdKreact(bpy.types.Operator):
    bl_idname = "md.k_react"
    bl_label = "K_react alpha"
    bl_description = "K_react alpha"
    bl_options = {'REGISTER', 'UNDO'}

    default_trig : bpy.props.BoolProperty(name="default_trig",
                            default=False,
                            description="Preview trig as default")

    mode  = MODE_FACE
    cycle = 0

    bm = None
    active_vertex  = None
    closest_vertex = None
    hold_line_vertex = None
    snap_line = None

    hit_normal = None
    hit_location = None

    pause = False
    hold_solution = False
    snap_mode = False

    object = None
    snap_obj = None
    undo_stack = []

    # [[edges], [vertex order list], created_face, [solution_score], face_type, closest_vertex]
    #     0             1                  2               3             4             5
    active_solution = [[], [], None, [-1.], FACE_NONE, None]

    mouse_x = -1
    mouse_y = -1

    last_time = 0
    delay_active = False

    timer = None

    '''
    def draw(self, context):
        layout = self.layout
        col = layout.column()
        col.prop(self, 'default_trig', text='Preview trig as default')
    '''

    def modal(self, context, event):
        #print("TIME: ", current_time())

        tooltip_text = 'Trig Mode: Shift | Line Mode: Shift + Ctrl | Hold: X | Undo: Z | Next solution: A | Prev solution: S | First solution: C'

        context.area.header_text_set(tooltip_text)

        if(event.alt and event.value=='PRESS'):
            self.delete_solution(self.active_solution)
            self.pause = True
        if(event.alt and event.value=='RELEASE'):
            self.last_time = current_time()
            self.pause = False

        if event.type in  {'MIDDLEMOUSE', 'WHEELDOWNMOUSE', 'WHEELUPMOUSE'}:
            # allow navigation
            return {'PASS_THROUGH'}
        else:

            if (event.type == 'SPACE' and event.value == 'PRESS'):
                context.object.data.update(True, True)
                self.cancel(context)
                return {'FINISHED'}

            elif event.type == 'MOUSEMOVE':
                self.cycle = 0
                return {'RUNNING_MODAL'}

            elif event.type == 'TIMER':
                if(self.pause == False):

                    if(self.delay_active and (current_time() - self.last_time) > SOLUTION_DELAY):
                        self.delay_active = False
                        self.last_time = current_time()

                    if(event.shift):
                        self.mode = MODE_TRIG
                    else:
                        self.mode = MODE_FACE

                    if(self.mouse_x == -1 and self.mouse_y == -1):
                        if(event.shift and event.ctrl):
                            try:
                                return self.K_react_line(context, event.mouse_region_x, event.mouse_region_y)
                            except Exception as e:
                                PrintException()
                                self.cancel(context)
                                return {'FINISHED'}
                        else:
                            #try:
                            return self.K_react(context, event.mouse_region_x, event.mouse_region_y)
                            #except Exception as e:
                            #    PrintException()
                            #    self.cancel(context)
                            #    return {'FINISHED'}
                    else:
                        dx = abs(event.mouse_region_x - self.mouse_x)
                        dy = abs(event.mouse_region_y - self.mouse_y)
                        if(dx > PIXEL_THRESHOLD or dy > PIXEL_THRESHOLD):
                            self.mouse_x = -1
                            self.mouse_y = -1

                return {'RUNNING_MODAL'}

            elif (event.type == 'LEFTMOUSE' and event.value == 'PRESS'):
                if(event.shift):
                    self.mode = MODE_TRIG
                else:
                    self.mode = MODE_FACE
                if(self.mouse_x == -1 and self.mouse_y == -1):
                    self.undo_stack.append(self.active_solution)
                    self.reset(reset_block = True, reset_snap = True)
                    self.mouse_x = event.mouse_region_x
                    self.mouse_y = event.mouse_region_y
                return {'RUNNING_MODAL'}

            elif (event.type == 'RIGHTMOUSE' and event.value == 'PRESS') or (event.type == 'ESC' and event.value == 'PRESS'):
                #print("0->")
                self.delete_solution(self.active_solution)
                self.cancel(context)
                return {'CANCELLED'}

            elif (event.type == 'A' and event.value == 'PRESS'):
                if(event.shift):
                    self.mode = MODE_TRIG
                else:
                    self.mode = MODE_FACE
                self.cycle += 1
                return {'RUNNING_MODAL'}

            elif (event.type == 'S' and event.value == 'PRESS'):
                if(event.shift):
                    self.mode = MODE_TRIG
                else:
                    self.mode = MODE_FACE
                if(self.cycle > 0):
                    self.cycle -= 1
                return {'RUNNING_MODAL'}

            elif (event.type == 'C' and event.value == 'PRESS'):
                self.cycle = 0
                return {'RUNNING_MODAL'}

            elif (event.type == 'X' and event.value == 'PRESS'):
                self.hold_solution = not self.hold_solution
                return {'RUNNING_MODAL'}

            elif (event.type == 'Z' and event.value == 'PRESS'):
                if(event.shift):
                    self.mode = MODE_TRIG
                else:
                    self.mode = MODE_FACE
                if(self.mouse_x == -1 and self.mouse_y == -1):
                    self.undo_stack.append(self.active_solution)
                    self.reset()
                    self.mouse_x = event.mouse_region_x
                    self.mouse_y = event.mouse_region_y
                self.undo()
                return {'RUNNING_MODAL'}

        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        #sys.stdout = open('C:/Users/K_tZg1/Dropbox/Sync/Scripts/k_tz_log.txt', 'w')

        #if(verbose): os.system('cls')
        if(bpy.context.object.mode != 'EDIT'):
            bpy.ops.object.mode_set(mode='EDIT')
        bpy.context.scene.tool_settings.use_snap = True
        bpy.context.scene.tool_settings.snap_elements = {'FACE'}

        self.timer = context.window_manager.event_timer_add(REFRESH_INTERVAL/1000., window=context.window)

        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        context.area.header_text_set(None)
        context.window_manager.event_timer_remove(self.timer)

    def undo(self):
        if(len(self.undo_stack) > 0):
            self.hold_solution = False
            self.delay_active  = False
            delete_sol = self.undo_stack.pop()
            #print("1->")
            self.delete_solution(delete_sol)

    def delete_solution(self, solution):
        #print("Delete ", solution)
        #print("> hv", self.hold_line_vertex)
        if(self.object != None):
            if(solution[4] != FACE_NONE):
                if(solution[4] == FACE_FILL):
                    bmesh.ops.delete(self.bm, geom=[solution[2]], context='ONLY_FACES')
                elif(solution[4] == FACE_SNAP):
                    delete_edge = [e for e in solution[2].edges if not(e in solution[0])]
                    bmesh.ops.delete(self.bm, geom=delete_edge, context='EDGES')
                elif(solution[4] == FACE_FREE):
                    delete_verts = []
                    existing_verts = []
                    new_verts      = []
                    for e in solution[2].edges:
                        if not(e in solution[0]):
                            new_verts.append(e.verts[0])
                            new_verts.append(e.verts[1])

                    for e in solution[2].edges:
                        if (e in solution[0]):
                            existing_verts.append(e.verts[0])
                            existing_verts.append(e.verts[1])

                    delete_verts =  list(set(new_verts) - set(existing_verts))
                    bmesh.ops.delete(self.bm, geom=delete_verts, context='VERTS')
                elif(solution[4] == FACE_LINK):
                    if(len(solution[0]) == len(solution[2].edges)):
                        bmesh.ops.delete(self.bm, geom=[solution[2]], context='FACES_ONLY')
                    else:
                        delete_edge = [e for e in solution[2].edges if not(e in solution[0])]
                        bmesh.ops.delete(self.bm, geom=delete_edge, context='EDGES')
                elif(solution[4] == FACE_LINE):
                    if(solution[0] != []):
                        bmesh.ops.delete(self.bm, geom=solution[0], context='EDGES')
                    elif(solution[1] != []):
                        bmesh.ops.delete(self.bm, geom=solution[1], context='VERTS')

                bmesh.update_edit_mesh(self.object.data)
                self.reset()

    def check_face_type(self, solution, face_type = FACE_NONE):
        if(self.snap_mode):
            # check if fill and link
            ns = len(solution[1])
            for i in range(ns):
                add_e = list(set(solution[1][i].link_edges).intersection(set(solution[1][(i+1)%ns].link_edges)))
                if(add_e != []):
                    if(not(add_e[0] in solution[0])): solution[0].append(add_e[0])

            if(self.hold_solution or self.delay_active):
                if(face_type == FACE_NONE): face_type = FACE_LINK
            if(len(solution[0]) == 4):
                if(face_type == FACE_NONE): face_type = FACE_FILL
            if(face_type == FACE_NONE): face_type = FACE_SNAP

        if(face_type == FACE_NONE): face_type = FACE_FREE
        return face_type


    def create_face(self, solution, face_type = FACE_NONE):
        if(len(solution[0]) < 3 and self.active_vertex == None and not self.snap_mode):
            self.active_vertex = self.bm.verts.new(self.hit_location)
            if(solution[1] != []):
                solution[1][0] = self.active_vertex

        face_type = self.check_face_type(solution, face_type)

        if(len(solution[1]) > 2):
            try:
                solution[2] = self.bm.faces.new(solution[1])
            except Exception as e:
                if(str(e) == 'faces.new(verts): face already exists'):
                    cf = None
                    for i in range(len(solution[1])-1):
                        t = set(solution[1][i].link_faces).intersection(solution[1][i+1].link_faces)
                        if(cf == None):
                            cf = t
                        else:
                            cf = cf.intersection(t)
                    cf = list(cf)
                    if(cf != []):
                        solution[2] = cf[0]
                        face_type = self.check_face_type(solution, face_type)

            solution[4] = face_type
            if(face_type != FACE_LINE):
                self.hold_line_vertex = None

        '''
        elif(len(solution[1]) == 2):
            face_type = FACE_NONE
            if(solution[0] == []):
                try:
                    solution[0].append(self.bm.edges.new(solution[1]))
                except:
                    print("[ERROR] > Line exists!")
        '''
        self.active_solution = solution

        self.bm.normal_update()

        bmesh.update_edit_mesh(self.object.data)

    # Get closest vertex #######################################################
    def get_closest_vertex(self):

        if hasattr(self.bm.verts, "ensure_lookup_table"):
            self.bm.verts.ensure_lookup_table()

        kd = kdtree.KDTree(len(self.bm.verts))
        for i, v in enumerate(self.bm.verts):
            kd.insert(v.co, i)
        kd.balance()
        kd_closest_vertex = kd.find_n(self.hit_location, MAX_SEARCH)

        #print("kd_closest_vertex ", kd_closest_vertex)
        #print(">Av ", self.active_vertex)
        neighbors = []
        for c in kd_closest_vertex:
            v = self.bm.verts[c[1]]
            if (len(self.bm.verts) == 1 or v.is_wire or v.is_boundary or v in self.active_solution[1]) and (v != self.active_vertex):
                # 0 index, 1 distance, 2 hit distance, 3 score
                neighbors.append([v, c[2], 0., [1.]])

        #print("neighbors ", neighbors)

        # Sort
        n = float(len(neighbors))
        if(n > 1):
            max_hit_distance = -1.
            #min_hit_distance = float('inf')

            #max_distance = -1.
            #min_distance = float('inf')

            for i, v in enumerate(neighbors):

                c_hit_distance, c_hit_normal = get_closest_ray_hit_test(v[0], self.object, self.snap_obj, self.hit_normal)
                #print("c_hit_distance", c_hit_distance)
                if(c_hit_normal != None):
                    v[3].append( (n - i) / n)
                    v[2] = c_hit_distance + SNAP_TOLERANCE

                    hit_conform = 1
                    if(not v[0].is_wire):
                        conform_angle = v[0].normal.angle(c_hit_normal, pi)
                        hit_conform = (-(conform_angle / pi)**2 + 1) / 2 + 0.5
                    v[3].append(hit_conform)

                    conform = 1
                    if(not v[0].is_wire):
                        conform_angle = v[0].normal.angle(self.hit_normal, pi)
                        conform = (-(conform_angle / pi)**2 + 1) / 2 + 0.5
                    v[3].append(conform)

                    #if(v[1] > max_distance): max_distance = v[1]
                    #if(v[1] < min_distance): min_distance = v[1]

                    if(v[2] > max_hit_distance): max_hit_distance = v[2]
                    #if(v[2] < min_hit_distance): min_hit_distance = v[2]
                else:
                    v[3][0] = 0


            for v in neighbors:
                if ( (1. - v[2]/max_hit_distance) > 0.5):
                    v[3].append( 1.)
                else:
                    v[3].append(.5)

                #v[3].append(v[1] / max_distance)

            neighbors.sort(key=lambda x: prod(x[3]), reverse=True)

        if (neighbors != []):
            #for v in neighbors:
                #try:
                #    print("--- v %5d "% v[0].index, "| [dist %.3f, conform %.3f, snap dist %.3f]"% (v[3][1], v[3][2], v[3][3]))
                #except:
                #    pass
            return (neighbors[0][0], neighbors[0][1])
        else:
            return (None, None)
    ############################################################################


    def reset(self, reset_block = False, reset_snap = False):
        if(reset_block):
            self.hold_solution = False
            self.delay_active  = False
        if(reset_snap):  self.snap_mode = False
        self.hold_line_vertex = self.active_vertex
        self.active_vertex = None
        self.closest_vertex = None
        self.snap_line = None
        self.active_solution = [[], [], None, [-1.], FACE_NONE, None]

    # Scoring ##########################################################
    def score_solution(self, potential_solution):

        def proj_point(a, b, p):
            ap = p-a
            ab = b-a
            dotabab = ab.dot(ab)
            if(dotabab == 0):
                result = a + ab
            else:
                result = a + ap.dot(ab)/ dotabab * ab
            return result

        solution = potential_solution[1]

        vectors = []
        len_vectors = len(solution)
        for i in range(len_vectors):
            vectors.append(Vector(solution[(i+1)%len_vectors].co) - Vector(solution[i].co))

        face_center = Vector((0,0,0))
        for tmp_v in solution:
            face_center += tmp_v.co
        face_center /= len(solution)

        trig_normal = []
        for i in range(len_vectors):
            trig_normal.append(vectors[i-1].cross(vectors[i]))

        # Check for Twist
        no_twist = 1
        if(self.mode == MODE_FACE):
            twist_angle1 = trig_normal[0].angle(trig_normal[2], pi)
            twist1 = (twist_angle1 / pi * 2)**4 / 16

            twist_angle2 = trig_normal[1].angle(trig_normal[3], pi)
            twist2 = (twist_angle2 / pi * 2)**4 / 16

            no_twist = 1 - twist1 * twist2

        # Check conform to normal
        conform = 0
        face_normal = Vector((0,0,0))
        for tmp_nrm in trig_normal:
            face_normal = (face_normal + tmp_nrm).normalized()

        conform_angle = face_normal.angle(self.hit_normal, pi)
        conform =  - (conform_angle / pi)**2 + 1

        # oposing edges
        oposing_edges = 1
        if(self.mode == MODE_FACE):
            op_edges_angle1 = vectors[0].angle(vectors[2], 0)
            op_edges_angle2 = vectors[1].angle(vectors[3], 0)
            oposing_edges = (atan(10*(op_edges_angle1-pi/2)) / pi + 0.5) * (atan(10*(op_edges_angle2-pi/2)) / pi + 0.5)

        # Pretty square
        pretty_square = 1
        if(self.mode == MODE_FACE):
            close_angle = pi - vectors[3].angle(vectors[0], 0)
            opose_angle = pi - vectors[1].angle(vectors[2], 0)

            close_square = -((close_angle % pi)/(pi / 2) -1)**2 + 1
            opose_square = -((opose_angle % pi)/(pi / 2) -1)**2 + 1

            pretty_square = close_square * opose_square
        elif(self.mode == MODE_TRIG):
            close_angle = pi - vectors[-1].angle(vectors[0], 0)

            pretty_square = -((2*close_angle - pi)/pi )**2  + 1

        # overlap faces with neighbor face
        no_overlap = 1
        for neighbor_edge in potential_solution[0]:
            if(neighbor_edge != None and len(neighbor_edge.link_faces) > 0):

                if(neighbor_edge.link_faces[0] == self.active_solution[2]): # ignore active face
                    if(len(neighbor_edge.link_faces) > 1):
                        neighbor_face = neighbor_edge.link_faces[1]
                    else:
                        continue
                else:
                    neighbor_face = neighbor_edge.link_faces[0]

                neighbor_face_center = Vector((0,0,0))
                for tmp_v in neighbor_face.verts:
                    neighbor_face_center += tmp_v.co
                neighbor_face_center /= len(neighbor_face.verts)

                proj_neighbor_face_center = proj_point(neighbor_edge.verts[0].co, neighbor_edge.verts[1].co, neighbor_face_center)
                proj_face_center          = proj_point(neighbor_edge.verts[0].co, neighbor_edge.verts[1].co, face_center)

                v0 = face_center - proj_face_center
                v1 = neighbor_face_center - proj_neighbor_face_center

                no_overlap_angle = v0.angle(v1, 0)
                #print("Overlap angle: ", degrees(no_overlap_angle), " face: ", neighbor_face.index)

                no_overlap *= -((no_overlap_angle-pi) / pi) ** 6 + 1


        potential_solution[3] = [conform, no_twist, oposing_edges, no_overlap, pretty_square]

        '''
        if(False):
            print("Sol: >", [e.index for e in potential_solution[0] if e != None], [s.index for s in solution])
            print("No twist   : %.6f" % no_twist)
            print("Conform    : %.6f" % conform)
            print("Square     : %.6f" % pretty_square)
            print("Opose      : %.6f" % oposing_edges)
            print("No overlap : %.6f" % no_overlap)
            print("SCORE      : %.6f <<<<" % prod(potential_solution[3]))
            print("")
        '''

        return potential_solution
    ####################################################################

    def K_react_line(self, context, x, y):
        self.hit_location, self.hit_normal, self.snap_obj, snap_obj_face_index = get_ray_hit(context, x, y)

        if(self.hit_location != None):
            #print("Line")

            bpy.ops.mesh.select_all(action='DESELECT')

            self.object = context.object

            if(self.bm == None): self.bm = bmesh.from_edit_mesh(self.object.data)

            if(len(self.bm.verts) == 0):
                self.active_vertex = None
                self.hold_line_vertex = self.bm.verts.new(self.hit_location)
                bmesh.update_edit_mesh(self.object.data)
                #return {'RUNNING_MODAL'}

            if(self.active_solution[4] != FACE_LINE and self.active_solution[4] != FACE_NONE):
                #print("2->")
                self.delete_solution(self.active_solution)
                self.hold_line_vertex = None

            closest_vertex , closest_vertex_distance = self.get_closest_vertex()

            #print("Av ", self.active_vertex)
            #print("Hv ", self.hold_line_vertex)
            #print("Cv ", closest_vertex, closest_vertex_distance)
            #print("As ", self.active_solution)

            if(len(closest_vertex.link_edges) > 0):
                average_edge_length = sum([e.calc_length() for e in closest_vertex.link_edges]) / float(len(closest_vertex.link_edges))
            else:
                average_edge_length = 0
            if(closest_vertex_distance < average_edge_length * SNAP_RATIO):
                if(self.hold_line_vertex != None and  not self.snap_mode and self.active_vertex != None
                    and closest_vertex != self.hold_line_vertex and
                    list(set(closest_vertex.link_edges).intersection(set(self.hold_line_vertex.link_edges))) == []):
                    bmesh.ops.delete(self.bm, geom=[self.active_vertex], context='VERTS')
                    self.active_vertex = None

                    self.snap_line = self.bm.edges.new([closest_vertex, self.hold_line_vertex])
                    bmesh.update_edit_mesh(self.object.data)

                    self.active_solution =  [[self.snap_line], [], None, [-1.], FACE_LINE, None]
                    self.snap_mode = True
            else:
                if(self.snap_mode):
                    if(self.snap_line != None):
                        bmesh.ops.delete(self.bm, geom=[self.snap_line], context='EDGES')
                    self.active_vertex = self.bm.verts.new(self.hit_location)
                    new_line = self.bm.edges.new([self.active_vertex, self.hold_line_vertex])
                    bmesh.update_edit_mesh(self.object.data)
                    self.snap_mode = False
                    self.active_solution =  [[], [self.active_vertex], None, [-1.], FACE_LINE, None]

                if(self.active_vertex == None):
                    #print(">>")
                    self.active_vertex = self.bm.verts.new(self.hit_location)

                    if(self.hold_line_vertex == None):
                        self.hold_line_vertex = closest_vertex

                    try:
                        self.bm.edges.new([self.active_vertex, self.hold_line_vertex])
                    except:
                        self.hold_line_vertex = closest_vertex
                        self.bm.edges.new([self.active_vertex, self.hold_line_vertex])

                    bmesh.update_edit_mesh(self.object.data)
                    self.active_solution =  [[], [self.active_vertex], None, [-1.], FACE_LINE, None]
                    #print("As ", self.active_solution)

                else:
                    self.active_vertex.co = self.hit_location
                    bmesh.update_edit_mesh(self.object.data)
        return {'RUNNING_MODAL'}


    def K_react(self, context, x, y):

        def deepcopy(source):
            return [[e for e in source[0]], [v for v in source[1]], source[2], [s for s in source[3]], source[4], source[5]]

        def same_solution( a, b):

            edges_a = [e.index for e in a[0] if e != None]
            edges_b = [e.index for e in b[0] if e != None]

            if(edges_a == [] or edges_b == []):
                return False

            vtx_a = [v for v in a[1]]
            vtx_b = [v for v in b[1]]

            if(edges_a == edges_b and vtx_a == vtx_b):
                return True
            else:
                return False

        self.hit_location, self.hit_normal, self.snap_obj, snap_obj_face_index = get_ray_hit(context, x, y)

        if(self.hit_location != None):
            #print("Face")

            #self.snap_obj.select = False

            bpy.ops.mesh.select_all(action='DESELECT')

            self.object = context.object

            if(self.bm == None): self.bm = bmesh.from_edit_mesh(self.object.data)

            solutions = []

            if((self.hold_solution or self.delay_active) and self.active_solution[1] == []):
                self.hold_solution = False
                self.delay_active = False

            if((self.mode == MODE_TRIG and len(self.bm.edges) < 2) or (self.mode == MODE_FACE and len(self.bm.edges) < 3)):
                #print("> line ")
                self.K_react_line(context, x, y)
            else:
                #print("> face ", len(self.bm.edges))

                if(self.active_solution[4] == FACE_LINE):
                    #print("3->")
                    self.delete_solution(self.active_solution)
                    self.hold_line_vertex = None

                self.closest_vertex, closest_vertex_distance = self.get_closest_vertex()

                #if((not(self.closest_vertex.is_boundary or self.closest_vertex.is_wire)) and (not (self.closest_vertex in self.active_solution[1]))):
                if(self.closest_vertex == None):
                    if(not self.hold_solution):
                        #print("4->")
                        self.delete_solution(self.active_solution)
                    else:
                        if((self.active_solution[4] == FACE_FREE or self.active_solution[4] == FACE_LINE) and self.active_vertex != None):
                            self.active_vertex.co = self.hit_location

                    return {'RUNNING_MODAL'}

                # Check snap #######################################################
                average_edge_length = sum([e.calc_length() for e in self.closest_vertex.link_edges]) / float(len(self.closest_vertex.link_edges))

                if(closest_vertex_distance < average_edge_length * SNAP_RATIO):
                    self.snap_mode = True
                    self.active_vertex = None

                    if((self.hold_solution or self.delay_active) and self.active_solution[4] != FACE_LINK): # Hold Snap in
                        if(not self.closest_vertex in self.active_solution[1]):
                            solution_copy = deepcopy(self.active_solution)
                            solution_copy[1][0] = self.closest_vertex
                            #print("5->")
                            self.delete_solution(self.active_solution)
                            self.create_face(solution_copy)
                            self.closest_vertex = solution_copy[1][0]
                        else:
                            self.active_vertex = self.active_solution[1][0]
                else:
                    self.snap_mode = False

                    if(self.active_solution[1] == []): # brand    active_vertex
                        self.active_vertex = self.bm.verts.new(self.hit_location)
                        bmesh.update_edit_mesh(self.object.data)

                    if(self.active_solution[4] == FACE_LINK): # Hold Snap out
                        solution_copy = deepcopy(self.active_solution)
                        if((self.mode == MODE_TRIG and len(solution_copy[0]) > 1) and (self.mode == MODE_FACE and len(solution_copy[0]) > 2)):
                            solution_copy[0].pop()
                        #print("6->")
                        self.delete_solution(self.active_solution)
                        self.active_vertex  = self.bm.verts.new(self.hit_location)
                        solution_copy[1][0] = self.active_vertex
                        bmesh.update_edit_mesh(self.object.data)
                        self.create_face(solution_copy, FACE_FREE)
                        self.closest_vertex = solution_copy[5]

                ####################################################################

                # Potential solutions ##############################################
                def get_solutions():
                    solutions = []
                    closest_vertices = []

                    forbidden_edges = []
                    forbidden_verts = []
                    if(self.active_solution[2] != None):
                        forbidden_edges = [e for e in self.active_solution[2].edges if not(e in self.active_solution[0])]
                        existing_verts = []
                        new_verts      = []
                        for e in self.active_solution[2].edges:
                            if not(e in self.active_solution[0]):
                                new_verts.append(e.verts[0])
                                new_verts.append(e.verts[1])

                        for e in self.active_solution[2].edges:
                            if (e in self.active_solution[0]):
                                existing_verts.append(e.verts[0])
                                existing_verts.append(e.verts[1])

                        forbidden_verts =  list(set(new_verts) - set(existing_verts))

                    if(self.mode == MODE_FACE):
                        if(self.snap_mode): depth = 3
                        else:               depth = 2
                    else:
                        if(self.snap_mode): depth = 2
                        else:               depth = 1

                    def crawl_edge(vertex, stack, level, tolerance=1):
                        if(level < depth):
                            for e in vertex.link_edges:
                                if((e.is_wire or e.is_boundary or e in self.active_solution[0]) and
                                        not (e in forbidden_edges) and not (e.verts[0] in forbidden_verts or e.verts[1] in forbidden_verts)):
                                    for i in range(2):
                                        new_stack = [stack[0].copy(), stack[1].copy()]
                                        if(e.verts[i] in stack[1]):
                                            next_tolerance = tolerance - 1
                                        else:
                                            next_tolerance = tolerance
                                            if(level == 0): new_stack[1].append(e.verts[(i+1)%len(e.verts)])
                                            new_stack[1].append(e.verts[i])
                                            new_stack[0].append(e)
                                        if(next_tolerance > 0):
                                            crawl_edge(e.verts[i], new_stack, level + 1, next_tolerance)
                        else:
                            if(not stack[1][0] in stack[1][1:]):

                                n = len(stack[1])
                                if(self.snap_mode):
                                    new_sol_a = [stack[0], stack[1], None, [-1], FACE_NONE, self.closest_vertex]
                                    solutions.append(self.score_solution(new_sol_a))
                                    if(self.mode == MODE_FACE):
                                        new_sol_b = [stack[0], [stack[1][n-i-1]  for i in range(n)], None, [-1], FACE_NONE, self.closest_vertex]
                                        solutions.append(self.score_solution(new_sol_b))
                                    else:
                                        if(stack[1][0] == self.closest_vertex):
                                            new_sol_b = [stack[0], [stack[1][n-i-1]  for i in range(n)], None, [-1], FACE_NONE, self.closest_vertex]
                                            solutions.append(self.score_solution(new_sol_b))
                                else:
                                    new_sol_a = [stack[0], stack[1], None, [-1], FACE_NONE, self.closest_vertex]
                                    solutions.append(self.score_solution(new_sol_a))
                                    if(stack[1][1] == self.closest_vertex and self.mode == MODE_FACE):
                                        new_sol_b = [stack[0], [stack[1][0], stack[1][3], stack[1][2], stack[1][1]], None, [-1], FACE_NONE, self.closest_vertex]
                                        solutions.append(self.score_solution(new_sol_b))

                    if(self.snap_mode):
                        crawl_edge(self.closest_vertex, [[], []], 0)
                    else:
                        if(self.active_vertex != None):
                            crawl_edge(self.closest_vertex, [[], [self.active_vertex]], 0)
                        else:
                            crawl_edge(self.closest_vertex, [[], [self.closest_vertex]], 0)

                    solutions.sort(key=lambda x: prod(x[3]), reverse=True)


                    return solutions
                ####################################################################

                if(self.hold_solution or self.delay_active):
                    self.cycle = 0
                    solutions = [self.active_solution]
                else:
                    solutions = get_solutions()


                if(solutions != []):
                    winner = self.cycle % len(solutions)

                    if( not same_solution(self.active_solution, solutions[winner])):
                        #print("Change: " ,  [e.index for e in self.active_solution[0] if e != None], [v.index for v in self.active_solution[1] if v!= None])
                        self.delay_active  = True
                        self.current_time  = current_time()
                        #print("7->")
                        self.delete_solution(self.active_solution)
                        self.create_face(solutions[winner])
                    else:
                        if((self.active_solution[4] == FACE_FREE or self.active_solution[4] == FACE_NONE) and self.active_vertex != None):
                            self.active_vertex.co = self.hit_location
                            bmesh.update_edit_mesh(self.object.data)

                else:
                    if(self.active_solution[4] != FACE_NONE):
                        #print("8->")
                        self.delete_solution(self.active_solution)
                        bmesh.update_edit_mesh(self.object.data)

            ####################################################################

            if(self.active_vertex != None):
                self.active_vertex.select = True


        return {'RUNNING_MODAL'}


# Addon Prefernedces ###########################################################


################################################################################

def register():
    bpy.utils.register_class(MdKreact)
    '''
    global addon_keymaps
    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new('Mesh', space_type='EMPTY', region_type='WINDOW', modal=False)
    kmi = km.keymap_items.new('md.k_react', 'RIGHTMOUSE', 'PRESS', ctrl=True)
    addon_keymaps.append(km)
    '''

def unregister():
    bpy.utils.unregister_class(MdKreact)

    '''
    global addon_keymaps
    wm = bpy.context.window_manager
    for km in addon_keymaps:
        if km in wm.keyconfigs.addon.keymaps.values():
            wm.keyconfigs.addon.keymaps.remove(km)
    addon_keymaps.clear()
    '''


if __name__ == "__main__":
    register()
