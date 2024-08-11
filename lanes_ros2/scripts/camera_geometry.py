import numpy as np

def get_intrinsic_matrix(field_of_view_deg, image_width, image_height):
    # For our Carla camera alpha_u = alpha_v = alpha
    # alpha can be computed given the cameras field of view via
    # field_of_view_rad = field_of_view_deg * np.pi/180
    # alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.)
    # Cu = image_width / 2.0
    # Cv = image_height / 2.0

    "You can use the below as the intrinsic matrix also instead of hardcoding the values"
    
    class Info(object):
        def __init__(self, dct):
            self.dct = dct

        def __getattr__(self, name):
            return self.dct[name]
    cameraInfo = Info({
        "focalLengthX": 262.8123779296875,   # focal length x 262.8860168457031
        "focalLengthY": 262.8123779296875,   # focal length y
        "opticalCenterX": 341.2982177734375, # optical center x 325.62530517578125
        "opticalCenterY": 182.76339721679688, # optical center y 175.42543029785156
        "cameraHeight": 1710,                # camera height in `mm` 
        "pitch": -30.0,                        # rotation degree around y
        "yaw": 0.0,                        # rotation degree around x
        "roll": 0.0                          # rotation degree around z
    })
    # c1 = cos(cameraInfo.pitch*pi/180)
    # s1 = sin(cameraInfo.pitch*pi/180)
    # c2 = cos(cameraInfo.yaw*pi/180)
    # s2 = sin(cameraInfo.yaw*pi/180)
    # KList = [
    #     [cameraInfo.focalLengthX * c2 + c1*s2* cameraInfo.opticalCenterX,
    #     -cameraInfo.focalLengthX * s2 + c1*c2* cameraInfo.opticalCenterX,
    #     -s1 * cameraInfo.opticalCenterX],
    #     [s2 * (-cameraInfo.focalLengthY * s1 + c1* cameraInfo.opticalCenterY),
    #     c2 * (-cameraInfo.focalLengthY * s1 + c1* cameraInfo.opticalCenterY),
    #     -cameraInfo.focalLengthY * c1 - s1* cameraInfo.opticalCenterY],
    #     [c1*s2, c1*c2, -s1]
    # ]

    KList = np.array([
        [cameraInfo.focalLengthX, 0, cameraInfo.opticalCenterX],
        [0, cameraInfo.focalLengthY, cameraInfo.opticalCenterY],
        [0, 0, 1]
    ])

    return KList


class CameraGeometry(object):
    def __init__(self, height=1.71, yaw_deg=0, pitch_deg=-30, roll_deg=0, image_width=640, image_height=480, field_of_view_deg=60):
        # scalar constants
        self.height = height
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.yaw_deg = yaw_deg
        self.image_width = image_width
        self.image_height = image_height
        self.field_of_view_deg = field_of_view_deg
        # camera intriniscs and extrinsics
        self.intrinsic_matrix = get_intrinsic_matrix(field_of_view_deg, image_width, image_height)
        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)
        ## Note that "rotation_cam_to_road" has the math symbol R_{rc} in the book
        yaw = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        roll = np.deg2rad(roll_deg)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rotation_road_to_cam = np.array([[cr*cy+sp*sr+sy, cr*sp*sy-cy*sr, -cp*sy],
                                            [cp*sr, cp*cr, sp],
                                            [cr*sy-cy*sp*sr, -cr*cy*sp -sr*sy, cp*cy]])
        self.rotation_cam_to_road = rotation_road_to_cam.T # for rotation matrices, taking the transpose is the same as inversion
        self.translation_cam_to_road = np.array([0,-self.height,0])
        self.trafo_cam_to_road = np.eye(4)
        self.trafo_cam_to_road[0:3,0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3,3] = self.translation_cam_to_road
        # compute vector nc. Note that R_{rc}^T = R_{cr}
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array([0,1,0])

    def camframe_to_roadframe_multiple(self, vecs_in_cam_frame):
        vecs_in_cam_frame = np.array(vecs_in_cam_frame)
        rotated_vecs = np.dot(vecs_in_cam_frame, self.rotation_cam_to_road)
        translated_vecs = rotated_vecs + self.translation_cam_to_road
        return translated_vecs
    
    def uv_coordinates_to_roadXYZ_roadframe_iso8855(self, uv_coordinates):
        uv_coordinates = np.array(uv_coordinates)
        uv_homogeneous = np.column_stack([uv_coordinates, np.ones(uv_coordinates.shape[0])])
        # Calculate roadXYZ in camera frame
        Kinv_uv_hom = np.dot(self.inverse_intrinsic_matrix, uv_homogeneous.T)
        denominator = self.road_normal_camframe.dot(Kinv_uv_hom)
        r_camframe = (self.height * Kinv_uv_hom / denominator).T
        
        # Convert to road frame and ISO 8855 coordinates
        r_roadframe = self.camframe_to_roadframe_multiple(r_camframe)
        roadXYZ_roadframe_iso8855 = np.column_stack([r_roadframe[:, 2], -r_roadframe[:, 0], -r_roadframe[:, 1]])
        
        return roadXYZ_roadframe_iso8855
    
