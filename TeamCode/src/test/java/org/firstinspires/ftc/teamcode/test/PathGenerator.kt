package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplinePair
import org.firstinspires.ftc.teamcode.mmooover.kinematics.CubicSplineSolver
import org.junit.jupiter.api.Test
import java.io.ByteArrayOutputStream
import java.io.DataOutputStream
import java.io.File
import java.io.OutputStreamWriter
import java.nio.ByteBuffer

class PathGenerator {
    private fun getTestOutputDir(): File {
        val path = File(System.getProperty("user.dir")!!).resolve("build").resolve("outputs").resolve("paths")
        path.mkdirs()
        return path
    }

    @Test fun `Generate auto path #1`() {
        CubicSplineSolver.silent = false
        val pairs = mutableListOf(
            0.0 to 0.0,
            10.0 to 10.0,
            0.0 to 0.0
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
                out.writeInt(waypointList.size)
                val expectedSize = waypointList.size
                for ((i, value) in waypointList.withIndex()) {
                    if (i >= expectedSize) throw IndexOutOfBoundsException("Too many values since writing size!")
                    out.writeDouble(value.x)
                    out.writeDouble(value.y)
                }
            }
        }
    }

    @Test fun `path test`() {
        println(getTestOutputDir())
    }
}