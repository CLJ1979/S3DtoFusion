from __future__ import annotations

from typing import List

import adsk
import adsk.core
import adsk.fusion
from adsk.core import Point3D

from ...s3dModel.types.bezier3d import Bezier3D
from ..converterSettings import ConverterSettings
from .utils import BezierPlane

# takes a sketch, bezier3d (the bezier3d must be a 2d bezier, all points on a plane) and a settings object
# creates the needed splines to represent the bezier


def bezier2dToCtrlPoints(bezier: Bezier3D, bezierPlane: BezierPlane, settings: ConverterSettings) -> List[List[List[float]]]:
    result = []

    for i in range(bezier.controlPoints.numberOfPoints-1):
        startPoint = bezier.controlPoints.points[i]
        endPoint = bezier.controlPoints.points[i+1]
        t1 = bezier.tangents1.points[i+1]
        t2 = bezier.tangents2.points[i]

        if bezierPlane == BezierPlane.XY:
            if settings.zUp:
                p1x, p1y = startPoint.x, startPoint.y
                p2x, p2y = t2.x, t2.y
                p3x, p3y = t1.x, t1.y
                p4x, p4y = endPoint.x, endPoint.y
            else:
                p1x, p1y = startPoint.x, startPoint.y
                p2x, p2y = t2.x, t2.y
                p3x, p3y = t1.x, t1.y
                p4x, p4y = endPoint.x, endPoint.y
            pos = startPoint.z

        elif bezierPlane == BezierPlane.XZ:
            if settings.zUp:
                p1x, p1y = startPoint.x, -startPoint.z
                p2x, p2y = t2.x, -t2.z
                p3x, p3y = t1.x, -t1.z
                p4x, p4y = endPoint.x, -endPoint.z
            else:
                p1x, p1y = startPoint.x, startPoint.z
                p2x, p2y = t2.x, t2.z
                p3x, p3y = t1.x, t1.z
                p4x, p4y = endPoint.x, endPoint.z
            pos = startPoint.y

        else:
            if settings.zUp:
                p1x, p1y = -startPoint.z, -startPoint.y
                p2x, p2y = -t2.z, -t2.y
                p3x, p3y = -t1.z, -t1.y
                p4x, p4y = -endPoint.z, -endPoint.y
            else:
                p1x, p1y = -startPoint.y, startPoint.z
                p2x, p2y = -t2.y, t2.z
                p3x, p3y = -t1.y, t1.z
                p4x, p4y = -endPoint.y, endPoint.z
            pos = startPoint.x

        result.append([[p1x, p1y], [p2x, p2y], [p3x, p3y], [p4x, p4y]])
    return result, pos


def bezier2dToSketch(sketch: adsk.fusion.Sketch, bezier: Bezier3D, bezierPlane: BezierPlane, settings: ConverterSettings):
    ctrl_points, _ = bezier2dToCtrlPoints(bezier, bezierPlane, settings)

    for cp in ctrl_points:
        points: List[Point3D] = [Point3D.create(p[0], p[1], 0) for p in cp]
        sketch.sketchCurves.sketchControlPointSplines.add(points, adsk.fusion.SplineDegrees.SplineDegreeThree)  # type: ignore

    adsk.doEvents()