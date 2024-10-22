package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplinePair
import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplineSolver
import org.junit.jupiter.api.Test

class SplineTest {
    @Test fun `test spline individual segments`() {
        CubicSplineSolver.solveMat(1, 1, 5, 5, 2, -1)
        CubicSplineSolver.solveMat(5, 5, 10, 5, -1, -5)
    }

    @Test fun `test spline multiple segments`() {
        val result = CubicSplineSolver.solveMultiSegment(
            doubleArrayOf(0.0, 5.0, 15.0, 10.0, 17.0),
            doubleArrayOf(0.0, 3.0, 0.0, 5.0, 10.0)
        )
        for (i in result) println(i.toDesmos())
    }

    @Test fun `test spline 2d multiple segments`() {
        val result = CubicSplineSolver.solve2DMultiSegment(
            doubleArrayOf(0.0, 5.0, 15.0, 10.0, 17.0),
            doubleArrayOf(0.0, 3.0, 0.0, 5.0, 10.0)
        )
        for (i in result) {
            println(i.toDesmos())
            i.computeWaypoints()
        }
    }

    @Test fun `Test large-scale`() {
        val result = CubicSplineSolver.solve2DMultiSegment(
            doubleArrayOf(0.0, 48.0, 96.0, 96.0, 96.0),
            doubleArrayOf(0.0, 0.0, 48.0, 96.0, 120.0)
        )
        for (i in result) {
            println(i.toDesmos())
        }
        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
        for (i in result)
            waypointList.addAll(i.computeWaypoints())
        println(waypointList)
        println("desmos: ${waypointList.map{it.toDesmos()}}")
    }

    @Test fun `Test small-scale`() {
        val result = CubicSplineSolver.solve2DMultiSegment(
            doubleArrayOf(0.0, 8.0, 8.0, 0.0, 0.0),
            doubleArrayOf(0.0, 0.0, 8.0, 16.0, 0.0)
        )
        for (i in result) {
            println(i.toDesmos())
        }
        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
        for (i in result)
            waypointList.addAll(i.computeWaypoints(0.5))
        println(waypointList)
        println("desmos: ${waypointList.map{it.toDesmos()}}")
    }
}