package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplinePair
import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplineSolver
import org.junit.jupiter.api.Test
import kotlin.random.Random
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

class SplineTest {
    @Test fun `test spline individual segments`() {
        CubicSplineSolver.silent = false
        CubicSplineSolver.solveMat(1, 1, 5, 5, 2, -1)
        CubicSplineSolver.solveMat(5, 5, 10, 5, -1, -5)
    }

    @Test fun `test spline multiple segments`() {
        CubicSplineSolver.silent = false
        val result = CubicSplineSolver.solveMultiSegment(
            doubleArrayOf(0.0, 5.0, 15.0, 10.0, 17.0),
            doubleArrayOf(0.0, 3.0, 0.0, 5.0, 10.0)
        )
        for (i in result) println(i.toDesmos())
    }

    @Test fun `test spline 2d multiple segments`() {
        CubicSplineSolver.silent = false
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
        CubicSplineSolver.silent = false
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

//    @Test fun `Test serializer`() {
//        CubicSplineSolver.silent = false
//
//        val result = CubicSplineSolver.solve2DMultiSegment(
//            doubleArrayOf(0.0, 48.0, 96.0, 96.0, 96.0),
//            doubleArrayOf(0.0, 0.0, 48.0, 96.0, 120.0)
//        )
//        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
//        for (i in result)
//            waypointList.addAll(i.computeWaypoints())
//
//        val roundtripWaypoints: MutableList<Waypoint>
//        ByteArrayOutputStream().use { outBytes ->
//            DataOutputStream(outBytes).use { out ->
//                CommandSerializer.serialize(waypointList, out)
//            }
//            val container = outBytes.toByteArray()
//            ByteArrayInputStream(container).use {inBytes ->
//                DataInputStream(inBytes).use {input ->
//                    roundtripWaypoints = WaypointSerializer.deserialize2(input)
//                }
//            }
//        }
//        for ((a, b) in waypointList.zip(roundtripWaypoints)) {
//            assert(a approx b) { "Roundtrip error" }
//        }
//    }

    @Test fun `Test fuzzing`() {
        CubicSplineSolver.silent = true
        val resultSet = mutableListOf<Double>()
        val iterCount = 100
        for (index in 1 .. iterCount){
            val x: MutableList<Double> = mutableListOf(0.0)
            val y: MutableList<Double> = mutableListOf(0.0)
            val rng = Random.Default
            repeat(10) {
                x.add(rng.nextDouble(-240.0, 240.0))
                y.add(rng.nextDouble(-240.0, 240.0))
            }
            x.add(0.0)
            y.add(0.0)

            val start = TimeSource.Monotonic.markNow()
            val result = CubicSplineSolver.solve2DMultiSegment(
                x.toDoubleArray(),
                y.toDoubleArray()
            )
            val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
            for (i in result)
                waypointList.addAll(i.computeWaypoints())
            val timeEnd = TimeSource.Monotonic.markNow()
            if (index % 10 == 0) println("%d completed of %d (%.2f%%)".format(index, iterCount, (index.toDouble())/iterCount*100))
            resultSet.add((timeEnd - start).toDouble(DurationUnit.MILLISECONDS))
        }
        println(
            "Min %.2f Avg %.2f Max %.2f (ms)".format(resultSet.min(), resultSet.average(), resultSet.max())
        )
    }

    @Test fun `Test small-scale`() {
        CubicSplineSolver.silent = false
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