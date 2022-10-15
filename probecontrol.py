import numpy as np
import polyscope as ps
import polyscope.imgui as psim
import pymeshlab
import time
import os
import glob

ANGLE_UNIT = np.pi/1920

def CREATE_BOUNDARIES(expr):
    ms = pymeshlab.MeshSet()
    ms.create_implicit_surface(expr=expr)
    return ms.current_mesh().vertex_matrix(),  ms.current_mesh().face_matrix()

class ViewPointMesh:
    def __init__(self, points, faces):
        self.points = points
        self.faces = faces


PARABOLA  = ViewPointMesh(CREATE_BOUNDARIES("x*x+y*y-z")[0],CREATE_BOUNDARIES("x*x+y*y-z")[1])
ELLIPSE   = ViewPointMesh(CREATE_BOUNDARIES("x*x+y*y+z*z")[0], CREATE_BOUNDARIES("x*x+y*y+z*z")[1])


square_points = np.array([[1/2,  1/2,1/2],
                          [1/2, -1/2,1/2],
                          [-1/2, 1/2,1/2],
                          [-1/2,-1/2,1/2],
                          [1/2,  1/2,-1/2],
                          [1/2, -1/2,-1/2],
                          [-1/2, 1/2,-1/2],
                          [-1/2,-1/2,-1/2]])

faces = np.array([[0, 1, 2],
                  [0, 1, 3],
                  [0, 2, 3],
                  [5, 2, 4],
                  [5, 2, 1],
                  [5, 1 ,4],
                  [6, 4, 3],
                  [6, 4, 1],
                  [6, 1, 3],
                  [7, 4, 3],
                  [7, 4, 2],
                  [7, 2, 3]])



SQUARE = ViewPointMesh(square_points, faces)





class ViewPoints:
    '''This is the new viewpoint class'''

    probes = []
    viewpointnum=0
    MousePosition = [0, 0]

    faces = np.array([[0, 1, 2],
                      [0, 1, 3],
                      [0, 2, 3],
                      [5, 2, 4],
                      [5, 2, 1],
                      [5, 1 ,4],
                      [6, 4, 3],
                      [6, 4, 1],
                      [6, 1, 3],
                      [7, 4, 3],
                      [7, 4, 2],
                      [7, 2, 3]])


    def __init__(self, x, y, z, dimension_x, dimension_y, dimension_z, Create_bounding_box = False):
        '''Constructor method'''
        ViewPoints.viewpointnum+=1
        self.viewpointnum = ViewPoints.viewpointnum

        self.x_position = x
        self.y_position = y
        self.z_position = z
        self.view_point = ps.register_point_cloud(f'view point {self.viewpointnum}', np.array([[x, y, z]]), color=(1,0,0.))
        self.Create_bounding_box = False

        self.length_x1 = dimension_x/2
        self.length_y1 = dimension_y/2
        self.length_z1 = dimension_z/2
        self.length_x2 = dimension_x/2
        self.length_y2 = dimension_y/2
        self.length_z2 = dimension_z/2

        self.speed = 1


        self.box_orientation_matrix = np.identity(4)

        self.points_bounding_box = self.draw_box()




        self.bounding_box = ps.register_surface_mesh(f"bounding box {self.viewpointnum}", self.points_bounding_box, ViewPoints.faces, transparency = 0.5, enabled= False)

        self.dimension_x = dimension_x
        self.dimension_y = dimension_y
        self.dimension_z = dimension_z
        self.Create_bounding_box = Create_bounding_box
        self.number_of_neighbours_normals = 50
        self.smoothiter = 3
        self.is_active=False

        ViewPoints.probes.append(self)



    def callback1(self):
        '''callbackpart'''

        if(psim.TreeNode(f"view point {self.viewpointnum}")):
            self.view_point.remove()
            self.is_active = True


            self.points_bounding_box = self.draw_box()

            # _, self.x_position = psim.SliderFloat('x', self.x_position, v_min = 0, v_max = self.dimension_x)
            # _, self.y_position = psim.SliderFloat('y', self.y_position, v_min = 0, v_max = self.dimension_y)
            # _, self.z_position = psim.SliderFloat('z', self.z_position, v_min = 0, v_max = self.dimension_z)




            if self.Create_bounding_box:

                self.bounding_box.remove()
                self.bounding_box = ps.register_surface_mesh(f"bounding box {self.viewpointnum}", self.points_bounding_box, ViewPoints.faces, color = (0, 0, 1), enabled=False, transparency=0.5)
                self.bounding_box.set_enabled(True)

                _, self.length_x1 = psim.SliderFloat('length x 1', self.length_x1, v_min = 0, v_max = self.dimension_x)
                _, self.length_y1 = psim.SliderFloat('length y 1', self.length_y1, v_min = 0, v_max = self.dimension_y)
                _, self.length_z1 = psim.SliderFloat('length z 1', self.length_z1, v_min = 0, v_max = self.dimension_z)

                _, self.length_x2 = psim.SliderFloat('length x 2', self.length_x2, v_min = 0, v_max = self.dimension_x)
                _, self.length_y2 = psim.SliderFloat('length y 2', self.length_y2, v_min = 0, v_max = self.dimension_y)
                _, self.length_z2 = psim.SliderFloat('length z 2', self.length_z2, v_min = 0, v_max = self.dimension_z)

                _, self.speed = psim.SliderFloat('Speed', self.speed, v_min=0, v_max=1)

            else:
                self.bounding_box.set_enabled(False)

            #Moving the viewpoint with the arrow keys
            vector = np.array([self.speed*(psim.IsKeyDown(262)-psim.IsKeyDown(263)),
                               self.speed*(psim.IsKeyDown(266)-psim.IsKeyDown(267)),
                               self.speed*(psim.IsKeyDown(265)-psim.IsKeyDown(264)),
                                                                         0])

            view_point = np.linalg.inv(ps.get_camera_view_matrix())

            delta_x = np.matmul(view_point, vector)
            self.x_position += delta_x[0]
            self.y_position += delta_x[1]
            self.z_position += delta_x[2]



            self.view_point = ps.register_point_cloud(f"view_point {self.viewpointnum}", np.array([[self.x_position, self.y_position, self.z_position]]), radius=0.005, color=(0, 1 ,0))
            self.view_point.add_vector_quantity(f"vector{self.viewpointnum}", np.array([np.array([0,0,0])-np.array([-1, 0, 0])]))

            self.get_roto_matrix_from_mouse()

        else:
            self.is_active= False
            self.view_point.set_color((1, 0 , 0))



    def draw_box(self):
        '''phi is the angle to the right, theta is the angle up'''

        x_position= self.x_position
        y_position= self.y_position
        z_position= self.z_position

        point = np.array([self.x_position, self.y_position, self.z_position])

        a1 = self.length_x1
        a2 = self.length_x2
        b1 = self.length_y1
        b2 = self.length_y2
        c1 = self.length_z1
        c2 = self.length_z2

        points_bounding_box = np.array([[-a2, -b2, -c2, 0],
                                        [+a1, -b2, -c2, 0],
                                        [-a2, +b1, -c2, 0],
                                        [-a2, -b2, +c1, 0],
                                        [+a1, +b1, +c1, 0],
                                        [+a1, +b1, -c2, 0],
                                        [+a1, -b2, +c1, 0],
                                        [-a2, +b1, +c1, 0]])

        new_points = np.transpose(np.matmul(self.box_orientation_matrix, np.transpose(points_bounding_box)))

        return new_points[:, :-1]+point


    def get_roto_matrix_from_mouse(self):
        '''This is the function that creates the correct RotoMatrix from the Mouse movement'''

        if not (psim.IsKeyDown(258)):
            ViewPoints.MousePosition = np.array(psim.GetMousePos())

        else:
            psim.CaptureMouseFromApp(True)
            change = np.array(psim.GetMousePos())-ViewPoints.MousePosition
            ViewPoints.MousePosition = psim.GetMousePos()
            angles = change*ANGLE_UNIT

            cvm = ps.get_camera_view_matrix()
            cvm_inv = np.linalg.inv(cvm)

            phi = angles[0]
            theta = angles[1]

            rotate_phi = np.array([[np.cos(-phi), 0, -np.sin(-phi),0],
                                   [0,           1,            0,0],
                                   [np.sin(-phi), 0,  np.cos(-phi),0],
                                   [0,           0,            0,1]])

            rotate_theta = np.array([[1 ,             0,               0, 0],
                                     [0 , np.cos(theta), -np.sin(theta) , 0],
                                     [0 , np.sin(theta), np.cos(theta) , 0],
                                     [0 ,             0,              0 , 1]])

            roto = np.matmul(rotate_theta, rotate_phi)
            rotation_matrix = np.matmul(cvm_inv, np.matmul(roto, cvm))
            self.box_orientation_matrix = np.matmul(rotation_matrix, self.box_orientation_matrix)


    def get_conditions_for_splitting(self, points):
        #get points in to 4-form
        points_trans = np.transpose(points)
        ones = np.full((1,points.shape[0]), 1)
        zeros = np.full((1, points.shape[0]),0)
        four_points = np.append(points_trans, ones, axis=0)
        #inverting the orientation_matrix
        inv_orient = np.linalg.inv(self.box_orientation_matrix)

        #bring the points in to the view_points_frame
        transformed_points = np.matmul(inv_orient, four_points)
        new_points = np.transpose(transformed_points[:-1])

        #bring the view_points in it's own frame
        view_point_position  = np.array([self.x_position, self.y_position, self.z_position, 1])
        view_point_own_frame = np.matmul(inv_orient, view_point_position)

        lower_x = view_point_own_frame[0]-self.length_x2
        lower_y = view_point_own_frame[1]-self.length_y2
        lower_z = view_point_own_frame[2]-self.length_z2


        upper_x = view_point_own_frame[0] + self.length_x1
        upper_y = view_point_own_frame[1] + self.length_y1
        upper_z = view_point_own_frame[2] + self.length_z1


        ms = pymeshlab.MeshSet()
        try:
            m  = pymeshlab.Mesh(new_points)
        except pymeshlab.pmeshlab.PyMeshLabException:
            ps.error('The Selected mesh is empty')
            return np.array([[0,0,0], [0,0,0]])

        ms.add_mesh(m)
        ms.compute_selection_by_condition_per_vertex(condselect = f"x < {upper_x} && x > {lower_x} && y < {upper_y} && y > {lower_y} && z < {upper_z} && z > {lower_z}")
        ms.generate_from_selected_vertices(deleteoriginal= False)
        ms.compute_normal_for_point_clouds(k = self.number_of_neighbours_normals, smoothiter = self.smoothiter, flipflag = True, viewpos = view_point_own_frame[:-1])


        #get back to the original frame
        points_ = ms.current_mesh().vertex_matrix()
        normals_= ms.current_mesh().vertex_normal_matrix()
        length = points_.shape[0]

        ones  = np.full((1,length),1)
        zeros = np.full((1,length),0)

        point_cloud_4 = np.append(np.transpose(points_),ones, axis = 0)
        with_normals_4 = np.append(np.transpose(normals_),zeros, axis = 0)
        point_cloud_4  = np.transpose(np.matmul(self.box_orientation_matrix, point_cloud_4))
        with_normals_4 = np.transpose(np.matmul(self.box_orientation_matrix, with_normals_4))

        point_cloud = point_cloud_4[:, :-1]
        with_normals = with_normals_4[:, :-1]


        return np.array([point_cloud, with_normals])

    def get_ranges(self):
        ranges = np.array([[self.length_x1, self.length_x2],
                           [self.length_y1, self.length_y2],
                           [self.length_z1, self.length_z2]])

        return ranges


    def set_ranges(self, ranges):
        try:
            self.length_x1 = ranges[0][0]
            self.length_x2 = ranges[0][1]

            self.length_y1 = ranges[1][0]
            self.length_y2 = ranges[1][1]

            self.length_z1 = ranges[2][0]
            self.length_z2 = ranges[2][1]





        except IndexError:
            print("The given range did not have the right shape")


    def get_dimensions(self):
        dimensions = np.array([self.dimension_x, self.dimension_y, self.dimension_z])
        return dimensions

    def set_dimensions(self, dimensions):
        try:
            self.dimension_x=dimensions[0]
            self.dimension_y=dimensions[1]
            self.dimension_z=dimensions[2]

        except IndexError:
            print("The given range did not have the right shape")



    def __del__(self):
        '''destructor method'''
        self.bounding_box.remove()
        self.view_point.remove()
        #ViewPoints.viewpointnum -= 1



    @classmethod
    def delete_active_view_points(cls):
        if len(cls.probes)==1:
            return
        for idx, point in enumerate(cls.probes):
            if not point.is_active:
                continue

            del cls.probes[idx]


    @classmethod
    def callback(cls):

        for point in cls.probes:
             point.callback1()

    @classmethod
    def calculate_normals(cls, MeshSet):
        pointcloud = np.expand_dims(np.array([0,0,0]), axis = 0)
        normals    = np.expand_dims(np.array([0,0,0]), axis = 0)
        for point in cls.probes:
            points_with_normals = point.get_conditions_for_splitting(MeshSet.current_mesh().vertex_matrix())
            points = points_with_normals[0]
            normals_= points_with_normals[1]
            pointcloud = np.append(pointcloud, points, axis = 0)
            normals = np.append(normals, normals_, axis = 0)

        return pointcloud, normals

    @classmethod
    def saveviewpointstofile(cls):
        '''This function saves all the viewpoints in the current directory'''
        if not os.path.exists("Viewpoints"):
            os.mkdir("Viewpoints")

        os.chdir("Viewpoints")

        for idx, point in enumerate(cls.probes):
            position=np.array([point.x_position, point.y_position, point.z_position])
            ranges = point.get_ranges()
            dimensions=point.get_dimensions()

            orientation_matrix = point.box_orientation_matrix
            np.savez(f"viewpointno{idx}", position=position, ranges=ranges, orientation_matrix=orientation_matrix, dimensions=dimensions)

        os.chdir("..")
        print("The viewpoints were saved in :" + os.getcwd())


    @classmethod
    def loadviewpoints(cls, folderpath):
        current_working_directory=os.getcwd()
        cls_old_view_point_number=cls.viewpointnum

        try:
            os.chdir(folderpath)
        except FileNotFoundError:
            ps.error(f"[Errno 2] No such file or directory: {folderpath}")
            return

        new_view_point_list = []

        for file in glob.glob("viewpointno*.npz"):
            viewpointfile=np.load(file)
            position = viewpointfile['position']
            ranges = viewpointfile['ranges']
            orientation_matrix = viewpointfile['orientation_matrix']
            dimensions = viewpointfile['dimensions']

            new_view_point = ViewPoints(x=position[0], y=position[1], z=position[2],
                                        dimension_x = dimensions[0], dimension_y = dimensions[1], dimension_z = dimensions[2], Create_bounding_box = True)
            print("The current view point number is:", cls.viewpointnum)

            new_view_point.orientation_matrix = orientation_matrix
            new_view_point.set_ranges(ranges)
            new_view_point_list.append(new_view_point)
            del new_view_point

        cls.probes = new_view_point_list
        cls.viewpointnum= cls.viewpointnum + cls_old_view_point_number
        print("The current view point number is:", cls.viewpointnum)





    @classmethod
    def remove_all(cls):
        while len(cls.probes)>0:
            del cls.probes[0]



# MESHSET=pymeshlab.MeshSet()
# CURRENT_DIRECTORY = os.getcwd()
# MESHSET.load_new_mesh(CURRENT_DIRECTORY+"/Experiment-78/"+"Experiment-78+pointcloud.ply")



# def main():
#     ps.init()
#     ps.look_at((0,0,1), (0,0,0))
#     ps.register_point_cloud("my point cloud", MESHSET.current_mesh().vertex_matrix(), radius = 0.001, point_render_mode='quad')
#     ViewPoints(0,0,0,15,15,15, True)
#     ps.set_user_callback(lambda MeshSet=MESHSET: ViewPoints.callback(MeshSet))
#     ps.show()
#
# if __name__ == '__main__':
#     main()
