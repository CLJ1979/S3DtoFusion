import numpy as np

from .bezier_board import BezierBoard

class BezierControlPointInterpolationSurfaceModel():

    def get_point_at(self, board: BezierBoard, x: float, s: float):
        interpolated_slice = board.get_interpolated_slice(x)

        if interpolated_slice is None:
            return (0.0, 0.0, 0.0)

        # min_s = BezierCurve.ONE
        # max_s = BezierCurve.ZERO

        # TODO: Handle min/max angle

        current_s = s #((max_s - min_s) * s) + min_s
        p_2d = interpolated_slice.get_point_at_s(current_s)

        return (x, p_2d[0], p_2d[1] + board.get_rocker_at_pos(x))
    
    # def get_derivative_at(self, board: BezierBoard, x: float, s: float):
    #     if x < 0.1:
    #         x = 0.1
    #     if x > board.get_length() - 0.1:
    #         x = board.get_length() - 0.1

    #     x_offset = 0.1
    #     s_offset = 0.01

    #     # flip_normal = False

    #     interpolated_slice = board.get_interpolated_slice(x)

    #     if interpolated_slice is None:
    #         return (0.0, 0.0, 0.0)
        
    #     # min_s = BezierCurve.ONE
    #     # max_s = BezierCurve.ZERO

    #     # TODO: Handle min/max angle

    #     current_s = s  #((max_s - min_s) * s) + min_s
    #     so = current_s + s_offset
    #     # if so > 1.0:
    #     #     so = current_s - s_offset
    #     #     flip_normal = True

    #     xo = x + x_offset
    #     interpolated_slice_xo = board.get_interpolated_slice(xo)
    #     if interpolated_slice_xo is None:
    #         return (0.0, 0.0, 0.0)

    #     # rocker_x = board.get_rocker_at_pos(x)
    #     # rocker_xo = board.get_rocker_at_pos(xo)

    #     p = interpolated_slice.get_point_at_s(current_s)
    #     p_so = interpolated_slice.get_point_at_s(so)
    #     p_xo = interpolated_slice_xo.get_point_at_s(current_s)

    #     # v_c = (0, p[0] - p_so[0], p[1] - p_so[1]) # Vector across
    #     # v_l = (xo - x, p_xo[0] - p[0], p_xo[1] - p[1] + rocker_xo - rocker_x) # Vector lengthwise
        
    #     df_dx = ((p[0] - p_xo[0])/x_offset, (p[1] - p_xo[1])/x_offset)
    #     df_ds = ((p[0] - p_so[0])/s_offset, (p[1] - p_so[1])/s_offset)
    #     # return df_dx, df_ds

    #     df_dy = 
    #     return df_dx, df_dy
    
    def get_normal_at(self, board: BezierBoard, x: float, s: float):

        if x < 0.1:
            x = 0.1
        if x > board.get_length() - 0.1:
            x = board.get_length() - 0.1

        x_offset = 0.1
        s_offset = 0.01

        flip_normal = False

        interpolated_slice = board.get_interpolated_slice(x)

        if interpolated_slice is None:
            return (0.0, 0.0, 0.0)
        
        # min_s = BezierCurve.ONE
        # max_s = BezierCurve.ZERO

        # TODO: Handle min/max angle

        current_s = s #((max_s - min_s) * s) + min_s
        so = current_s + s_offset
        if so > 1.0:
            so = current_s - s_offset
            flip_normal = True

        xo = x + x_offset
        interpolated_slice_xo = board.get_interpolated_slice(xo)
        if interpolated_slice_xo is None:
            return (0.0, 0.0, 0.0)

        rocker_x = board.get_rocker_at_pos(x)
        rocker_xo = board.get_rocker_at_pos(xo)

        p = interpolated_slice.get_point_at_s(current_s)
        p_so = interpolated_slice.get_point_at_s(so)
        p_xo = interpolated_slice_xo.get_point_at_s(current_s)

        v_c = (0, p[0] - p_so[0], p[1] - p_so[1]) # Vector across
        # //vc.normalize();

        v_l = (xo - x, p_xo[0] - p[0], p_xo[1] - p[1] + rocker_xo - rocker_x) # Vector lengthwise
        # //vl.normalize();
        # print(x, xo, s, so)
        # print(p, p_xo, rocker_x, rocker_xo)
        # print(v_c, v_l)

        normal_vec = np.cross(v_l, v_c)
        normal_vec = normal_vec / np.linalg.norm(normal_vec)
        if flip_normal:
            normal_vec *= -1
        return tuple(normal_vec)