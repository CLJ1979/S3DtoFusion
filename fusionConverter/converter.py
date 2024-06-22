# The Converter class takes a S3DModel and converts its content to Fusion360 Sketches
from typing import List, Optional, Set, Any

import adsk
import adsk.core
import adsk.fusion
from adsk.fusion import ConstructionPlane

from .. import config
from ..fusionConverter.loftHandler import LoftHandler
from ..fusionConverter.typeConverter.lines import (
    intersection_curve_via_surface_project,
    intersection_curve_via_text_command)
from ..s3dModel.s3dx import S3DModel
from ..s3dModel.types.point3d import Point3d
from ..s3dModel.types.bezier3d import Bezier3D
from ..utils import timer
from ..utils.transform import sketchPointListToPoint3DList
from .converterSettings import ConverterSettings
from .modelConverter import box as boxconverter
from .modelConverter import curveDef, outline
from .modelConverter import slice as sliceconverter
from .modelConverter import stringer
from .typeConverter.bezierToSketch import bezier2dToCtrlPoints
from .typeConverter.utils import BezierPlane


class Converter():
    model: S3DModel
    settings: ConverterSettings
    def __init__(self, model: S3DModel, settings: Optional[ConverterSettings]) -> None:
        self.model = model
        if settings is None:
            self.settings = ConverterSettings()
        else:
            self.settings = settings

    def convertAll(self, app: adsk.core.Application, rockerPlane: Optional[ConstructionPlane], progress: adsk.core.ProgressDialog):
        print("Start converting an creating sketches...")
        timer.reset()
        rootComponent = adsk.fusion.Design.cast(app.activeProduct).rootComponent
        component = rootComponent
        if rockerPlane is None:
            rootComponent = adsk.fusion.Design.cast(app.activeProduct).rootComponent
            if self.settings.zUp:
                rockerPlane = rootComponent.xZConstructionPlane
                # rockerPlane = rootComponent.
                tailPlane: ConstructionPlane = rootComponent.yZConstructionPlane
                outlinePlane: ConstructionPlane = rootComponent.xYConstructionPlane
            else:

                rockerPlane = rootComponent.xYConstructionPlane
                tailPlane = rootComponent.yZConstructionPlane
                outlinePlane: ConstructionPlane = rootComponent.xZConstructionPlane
        else:
            # TODO implement creation based on given rockerplane
            # create plane for tail slice
            tailPlane = None
            # create plane for Outline
            outlinePlane = None
            component = None
            raise NotImplementedError()

        if not config.debug_skip2d:
            progress.progressValue = 55
            progress.message = "Building Stringer"
            adsk.doEvents()
            self.convertStringer(component, rockerPlane)  # type: ignore
            progress.progressValue = 60
            progress.message = "Building Outline"
            adsk.doEvents()
            self.convertOutline(component, outlinePlane)  # type: ignore
            progress.progressValue = 65
            progress.message = "Building Slices"
            adsk.doEvents()
            self.convertSlices(component, tailPlane)  # type: ignore
            progress.progressValue = 70
            progress.message = "Building Misc Lines"
            adsk.doEvents()
            self.convertCurveDefs(component, outlinePlane, rockerPlane)  # type: ignore
            progress.progressValue = 75
            progress.message = "Building Boxes"
            adsk.doEvents()
            self.convertBoxes(component, outlinePlane)  # type: ignore
            progress.progressValue = 80

        progress.message = "Building 3D Lines"
        adsk.doEvents()
        if config.experimental_3d_spline_implementation == config.SplineImplementationTechnique.INTERPOLATE:
            print("Running interpolation spline mode....")
            self.create_3D_interpolation(component, outlinePlane)
        else:
            self.create3D(component, outlinePlane)  # type: ignore

        timer.lap()
        print("Done")

    def convertStringer(self, comp: adsk.fusion.Component, rockerPlane: ConstructionPlane):
        stringer.convert(comp, rockerPlane, self.model.board.strDeck, self.model.board.strBot, self.settings)

    def convertOutline(self, comp: adsk.fusion.Component, outlinePlane: ConstructionPlane):
        outline.convert(comp, outlinePlane, self.model.board.otl, self.settings)

    def convertCurveDefs(self, comp: adsk.fusion.Component, outlinePlane: ConstructionPlane, rockerPlane: ConstructionPlane):
        curveDef.convert(comp, outlinePlane, rockerPlane, self.model.board.curveDefTops, self.model.board.curveDefSides, self.settings)

    # runs the slice conversion for all slices
    def convertSlices(self, comp: adsk.fusion.Component, tailPlane: ConstructionPlane):
        i: int = 0
        for board_slice in self.model.board.slices:
            i += 1
            sliceconverter.convert(comp, tailPlane, board_slice, f"Slice {i}", self.settings)

    def convertBoxes(self, comp: adsk.fusion.Component, outlinePlane: ConstructionPlane):
        # for box in self.model.board.boxes:
        #     boxconverter.covert(app)
        #     pass
        boxconverter.convertEasy(comp, outlinePlane, self.model.board.boxes, self.settings)

    def create_3D_interpolation(self, comp: adsk.fusion.Component, outlinePlane: ConstructionPlane):
        from .cadcore.bezier_curve import BezierCurve
        from .cadcore.bezier_slice import BezierSlice
        from .cadcore.bezier_board import BezierBoard
        from .cadcore.bezier_control_point_interpolation_surface_model import BezierControlPointInterpolationSurfaceModel

        from adsk.fusion import ConstructionPlane, Sketch
        from adsk.core import Point3D

        def read_bezier_slice_from_model(bezier: Bezier3D, plane: BezierPlane):
            ctrl_points, pos = bezier2dToCtrlPoints(bezier, plane, self.settings)
            # TODO: Should we remove the last ctrl point?
            
            cc_bezier_curves = [BezierCurve(cp) for cp in ctrl_points]
            return BezierSlice(cc_bezier_curves, pos)

        # Read slices
        cc_bezier_slices = []
        for board_slice in self.model.board.slices:
            bezier = board_slice.bezier.getMirrored(False, True, False)
            
            # Subtract however high the slice is above zero
            height = bezier.controlPoints.points[0].z
            bezier.controlPoints.points = [Point3d.new(p.x, p.y, p.z - height) for p in bezier.controlPoints.points]
            bezier.tangents1.points = [Point3d.new(p.x, p.y, p.z - height) for p in bezier.tangents1.points]
            bezier.tangents2.points = [Point3d.new(p.x, p.y, p.z - height) for p in bezier.tangents2.points]

            cc_bezier_slice = read_bezier_slice_from_model(bezier, BezierPlane.YZ)
            cc_bezier_slices.append(cc_bezier_slice)

        # Read stringer and outline
        cc_bezier_deck = read_bezier_slice_from_model(self.model.board.strDeck, BezierPlane.XZ)
        cc_bezier_bottom = read_bezier_slice_from_model(self.model.board.strBot, BezierPlane.XZ)
        cc_bezier_outline = read_bezier_slice_from_model(self.model.board.otl, BezierPlane.XY)

        cc_board = BezierBoard(cc_bezier_slices, cc_bezier_deck, cc_bezier_bottom, cc_bezier_outline)

        # Interpolate 3D apex curves
        surface_model = BezierControlPointInterpolationSurfaceModel()
        apex_curves = cc_board.get_interpolated_apex_3d(surface_model)

        # Add 3d sketch for the apex
        apex_sketch: Sketch = comp.sketches.add(outlinePlane)
        apex_sketch.name = "Apex 3D"
        for curve in apex_curves:
            points: List[Point3D] = [Point3D.create(p[0], p[1], p[2]) for p in curve.ctrlpts]
            apex_sketch.sketchCurves.sketchControlPointSplines.add(points, adsk.fusion.SplineDegrees.SplineDegreeThree)
        # Add mirrored apex curve
        for curve in apex_curves:
            points: List[Point3D] = [Point3D.create(p[0], -p[1], p[2]) for p in curve.ctrlpts]
            apex_sketch.sketchCurves.sketchControlPointSplines.add(points, adsk.fusion.SplineDegrees.SplineDegreeThree)

    
        # Interpolate 3D slice control point curves (deck, rail, bottom, etc.)
        for i in range(len(cc_board.bezier_slices[0].curves) - 1):
            curves = cc_board.get_interpolated_curves_3d(surface_model, curve_idx=i)

            # Add 3d sketch for each slice control point curve
            curves_3d_sketch: Sketch = comp.sketches.add(outlinePlane)
            curves_3d_sketch.name = f"Slice curve 3D - {i}"
            for curve in curves:
                points: List[Point3D] = [Point3D.create(p[0], p[1], p[2]) for p in curve.ctrlpts]
                curves_3d_sketch.sketchCurves.sketchControlPointSplines.add(points, adsk.fusion.SplineDegrees.SplineDegreeThree)
            # Add mirrored curve
            for curve in curves:
                points: List[Point3D] = [Point3D.create(p[0], -p[1], p[2]) for p in curve.ctrlpts]
                curves_3d_sketch.sketchCurves.sketchControlPointSplines.add(points, adsk.fusion.SplineDegrees.SplineDegreeThree)
        

    def create3D(self, comp: adsk.fusion.Component, outlinePlane: ConstructionPlane):
        # Part Zero: Check if all needed Curves are present (Rail, Apex, Deck)
        # 3D
        nameList: List[str] = []
        doublesList: List[str] = []
        for i in range(comp.sketches.count):
            sketch = comp.sketches.item(i)
            name = sketch.name
            if name.find("(") == -1:
                continue
            sketch.isLightBulbOn = False
            name = name.replace(" (Top)", "").replace(" (Side)", "")
            if name in nameList:
                doublesList.append(name)
            else:
                nameList.append(name)

        # Part One: Create 3D Splines by using intersectioncurves
        for entry in doublesList:
            sideSketch = comp.sketches.itemByName(entry + " (Side)")
            sideSketch.isLightBulbOn = False
            sideSplines = sideSketch.sketchCurves.sketchControlPointSplines

            topSketch = comp.sketches.itemByName(entry + " (Top)")
            topSketch.isLightBulbOn = False
            topSplines = topSketch.sketchCurves.sketchControlPointSplines

            if config.experimental_3d_spline_implementation == config.SplineImplementationTechnique.TEXTCOMMANDS:
                sketch: adsk.fusion.Sketch = comp.sketches.add(outlinePlane)  # type: ignore
                sketch.name = entry + " 3D"
                intersection_curve_via_text_command(sideSplines, topSplines, sketch, False)
            
                # comp.sketches.itemByName(entry + " (Top)").sketchCurves.sketchControlPointSplines.add(side, 3)
                # check if Splines are reversed (don't know why this happens)
                splinePointsList: List[List[adsk.core.Point3D]] = []
                isReversed: bool = False
                # print(len(sketch.sketchCurves.sketchControlPointSplines))
                for spline in sketch.sketchCurves.sketchControlPointSplines:
                    points = sketchPointListToPoint3DList(spline.controlPoints)
                    if spline.startSketchPoint.geometry.x > spline.endSketchPoint.geometry.x:
                        # spline is revered -> the start point is further away from x=0 than the end point
                        # print(f"start {spline.startSketchPoint.geometry.x} -> end {spline.endSketchPoint.geometry.x}")
                        points.reverse()
                        isReversed = True
                    splinePointsList.append(points)

                # this flipps the reversed splines BUT the new creation of the splines also fixes some weird errors
                # so even if the splines were all correct, delete and create them new!

                for i in range(sketch.sketchCurves.sketchControlPointSplines.count -1, -1, -1):
                    sketch.sketchCurves.sketchControlPointSplines[i].deleteMe()

                for splinePoints in splinePointsList:
                    sketch.sketchCurves.sketchControlPointSplines.add(splinePoints, 3)  # type: ignore

                # make splines continuous
                for i in range(sketch.sketchCurves.sketchControlPointSplines.count - 1):
                    spline1 = sketch.sketchCurves.sketchControlPointSplines[i]
                    spline2 = sketch.sketchCurves.sketchControlPointSplines[i+1]
                    # set the startpoint to the endpoint of the last
                    spline2.startSketchPoint.merge(spline1.endSketchPoint)

                # constrain control lines from the splines so that the splines are continuous and smooth
                constraints = sketch.geometricConstraints
                lineIndex = -1
                for i in range(sketch.sketchCurves.sketchControlPointSplines.count - 1):
                    lineIndex += len(sketch.sketchCurves.sketchControlPointSplines[i].controlPoints) - 1
                    # sketch.sketchCurves.sketchLines[lineIndex].isFixed = False
                    constraints.addCollinear(sketch.sketchCurves.sketchLines[lineIndex], sketch.sketchCurves.sketchLines[lineIndex + 1])
            
            elif config.experimental_3d_spline_implementation == config.SplineImplementationTechnique.PROJECT_TO_SURFACE:
                sketch = intersection_curve_via_surface_project(sideSplines, topSplines, entry + " 3D")
                if config.settings_object.merge3dSplineEndpoints:
                    # define spline "start" as the point with the lower x value
                    splines = sketch.sketchCurves.sketchControlPointSplines
                    starts: List[adsk.fusion.SketchPoint] = []
                    ends: List[adsk.fusion.SketchPoint] = []
                    for sp in splines:
                        # first x is smaller than last x -> first is start
                        if sp.controlPoints[0].geometry.x < sp.controlPoints[len(sp.controlPoints)-1].geometry.x:
                            starts.append(sp.controlPoints[0])
                            ends.append(sp.controlPoints[len(sp.controlPoints)-1])
                        else:
                            starts.append(sp.controlPoints[len(sp.controlPoints)-1])
                            ends.append(sp.controlPoints[0])

                    for start in starts[1:]:
                        nearestPoint: adsk.fusion.SketchPoint = ends[0]
                        nearestDistance: float = 10000000000
                        for end in ends:
                            distance = start.geometry.distanceTo(end.geometry)
                            if distance < nearestDistance:
                                nearestDistance = distance
                                nearestPoint = end
                        start.merge(nearestPoint)
                        ends.remove(nearestPoint)

            else:
                raise NotImplementedError()
