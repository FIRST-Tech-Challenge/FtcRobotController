package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplinePair
import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplineSolver
import org.firstinspires.ftc.teamcode.mmooover.kinematics.PathHelper
import org.firstinspires.ftc.teamcode.mmooover.kinematics.WaypointSerializer
import org.firstinspires.ftc.teamcode.mmooover.kinematics.path
import org.junit.jupiter.api.Test
import java.io.DataOutputStream
import java.io.File
import kotlin.math.PI


class PathGenerator {
    private fun getTestOutputDir(): File {
        val path = File(System.getProperty("user.dir")!!).resolve("build").resolve("outputs")
            .resolve("paths")
        path.mkdirs()
        return path
    }

    private fun pair(a: Number, b: Number) = a.toDouble() to b.toDouble()

    @Test
    fun `Generate auto path #1`() {
        CubicSplineSolver.silent = false
        val pairs = mutableListOf(
            pair(0, 0),
            pair(0, 96.0),
            pair(-24, 96)
        )
        val xItems = pairs.map { it.first }.toDoubleArray()
        val yItems = pairs.map { it.second }.toDoubleArray()

        val solution = CubicSplineSolver.solve2DMultiSegment(xItems, yItems)
        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
        for (i in solution) {
            if (!waypointList.isEmpty()) waypointList.removeLast() // pop the duplicate waypoint
            waypointList.addAll(i.computeWaypoints())
        }

        val outFile = getTestOutputDir().resolve("auto1.bin")
        if (outFile.exists()) outFile.delete()
        outFile.outputStream().use {
            DataOutputStream(it).use { out ->
                WaypointSerializer.serialize2(waypointList, out)
            }
        }
    }

    @Test
    fun `generate auto path facing v2`() {
        val triads = path {
            k(0, 0, 0.deg)
            k(0, 96)
            k(180.deg) // Halfway between the previous and next keyframes
            k(48, 96, 90.deg)
        }.generate()

        val outFile = getTestOutputDir().resolve("auto2.bin")
        if (outFile.exists()) outFile.delete()
        outFile.outputStream().use {
            DataOutputStream(it).use { out ->
                WaypointSerializer.serialize3(triads, out)
            }
        }
    }

    @Test
    fun `Generate auto path with facing`() {
        CubicSplineSolver.silent = false
        val pairs = mutableListOf(
            0.0 to 0.0,
            10.0 to 10.0,
            0.0 to 0.0
        )
        val xItems = pairs.map { it.first }.toDoubleArray()
        val yItems = pairs.map { it.second }.toDoubleArray()

        val solution = CubicSplineSolver.solve2DMultiSegment(xItems, yItems)
        val facings = listOf(
            0.0 to 0.0,
            2.0 to PI / 2.0,
            3.0 to PI / 2.0,
            solution.last().tTo + solution.last().x.offset to PI / 2.0
        )
        val facingCurve =
            CubicSplineSolver.solveMultiSegment(
                facings.map { it.first }.toDoubleArray(),
                facings.map { it.second }.toDoubleArray()
            )
        val waypointList: MutableList<CubicSplinePair.PointData> = mutableListOf()
        for (i in solution) {
            if (!waypointList.isEmpty()) waypointList.removeLast() // pop the duplicate waypoint
            waypointList.addAll(i.computeWaypoints())
        }
        val zipped = CubicSplineSolver.thirdChannel(waypointList, facingCurve)

        val outFile = getTestOutputDir().resolve("auto1f.bin")
        if (outFile.exists()) outFile.delete()
        outFile.outputStream().use {
            DataOutputStream(it).use { out ->
                WaypointSerializer.serialize3(zipped, out)
            }
        }
    }

    @Test
    fun `path test`() {
        println(getTestOutputDir())
    }
}