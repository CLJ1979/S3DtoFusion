from typing import List
from geomdl import fitting, operations
import numpy as np

from .bezier_curve import BezierCurve, DEG_TO_RAD
from .bezier_slice import BezierSlice
from .fitting import interpolate_cubic_spline_curve_w_derivative, interpolate_local_cubic_curves

class BezierBoard():
    bezier_slices: List[BezierSlice]
    bezier_deck: BezierSlice
    bezier_bottom: BezierSlice
    bezier_outline: BezierSlice

    def __init__(self, bezier_slices: List[BezierSlice], bezier_deck: BezierSlice, 
                 bezier_bottom: BezierSlice, bezier_outline: BezierSlice):
        self.bezier_slices = bezier_slices
        self.bezier_deck = bezier_deck
        self.bezier_bottom = bezier_bottom
        self.bezier_outline = bezier_outline

    def get_nearest_slice_index(self, x: float):
        slice_positions = [s.position for s in self.bezier_slices]
        slice_distances = [abs(x - pos) for pos in slice_positions]
        min_dist = min(slice_distances)
        return slice_distances.index(min_dist)
    
    def get_length(self):
        return max([ctrlpt[0] for curve in self.bezier_outline.curves for ctrlpt in curve.ctrlpts])

    def get_deck_at_pos(self, pos: float):
        return self.bezier_deck.get_value_at(pos)

    def get_rocker_at_pos(self, pos: float):
        return self.bezier_bottom.get_value_at(pos)

    def get_thickness_at_pos(self, pos: float):
        return self.get_deck_at_pos(pos) - self.get_rocker_at_pos(pos)

    def get_width_at_pos(self, pos: float):
        return self.bezier_outline.get_value_at(pos) * 2

    def get_interpolated_slice(self, x: float):
        # Find nearest slices
        idx = self.get_nearest_slice_index(x)

        if self.bezier_slices[idx].position == x:
            return self.bezier_slices[idx]
        elif self.bezier_slices[idx].position > x:
            idx -= 1
        next_idx = idx + 1

        slice1 = self.bezier_slices[idx]
        slice2 = self.bezier_slices[next_idx]

        # Calculate t
        t = (x - slice1.position) / (slice2.position - slice1.position)

        # Interpolate slice
        interpolated_slice = slice1.interpolate(slice2, t)

        # Calculate scale
        thickness = self.get_thickness_at_pos(x)

        if thickness < 0.5:
            thickness = 0.5

        width = self.get_width_at_pos(x)

        if width < 0.5:
            width = 0.5

        interpolated_slice = interpolated_slice.scale(thickness, width)
        interpolated_slice.position = x

        return interpolated_slice
    
    def get_interpolated_apex_3d(self, surface_model, num_slice_interpolations: int = 2):
        slice_positions = [bs.position for bs in self.bezier_slices]

        APEX_DEFINITION_ANGLE = 90

        apex_points = []
        for min_x, max_x in zip(slice_positions[:-1], slice_positions[1:]):
        
            include_endpoint = True if max_x == max(slice_positions) else False

            for x in np.linspace(min_x, max_x, num_slice_interpolations, endpoint=include_endpoint):
                # Find apex
                interpolated_slice = self.get_interpolated_slice(x)
                s_for_apex = interpolated_slice.get_s_by_normal_reverse(APEX_DEFINITION_ANGLE * DEG_TO_RAD)
                p = surface_model.get_point_at(self, x, s_for_apex)
                apex_points.append(p)

        # Do local curve interpolation - NOTE: This results in a Bezier 3-dim curve
        curves = interpolate_local_cubic_curves(apex_points)
        
        return curves
    
    def get_interpolated_curves_3d(self, surface_model, curve_idx, num_slice_interpolations: int = 4):
        slice_positions = [bs.position for bs in self.bezier_slices]

        curve_points = []
        for min_x, max_x in zip(slice_positions[:-1], slice_positions[1:]):
            include_endpoint = True if max_x == max(slice_positions) else False

            for x in np.linspace(min_x, max_x, num_slice_interpolations, endpoint=include_endpoint):
                # Get control point of the interpolated slice
                interpolated_slice = self.get_interpolated_slice(x)

                p_2d = interpolated_slice.curves[curve_idx].ctrlpts[-1]
                p_3d = (x, p_2d[0], p_2d[1] + self.get_rocker_at_pos(x))
                curve_points.append(p_3d)

        # Do local curve interpolation - NOTE: This results in a Bezier 3-dim curve
        curves = interpolate_local_cubic_curves(curve_points)
        
        return curves