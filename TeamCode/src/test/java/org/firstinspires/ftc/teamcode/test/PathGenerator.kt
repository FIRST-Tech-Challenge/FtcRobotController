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

    @Test
    fun `generate blue right auto v1`() {
        val triads = path {
            k(0, 0, 0.deg)
            k(0, 48)
            k(48, 48, 90.deg)
        }.generate()

        val outFile = getTestOutputDir().resolve("AutoBlueRight1.bin")
        if (outFile.exists()) outFile.delete()
        outFile.outputStream().use {
            DataOutputStream(it).use { out ->
                WaypointSerializer.serialize3(triads, out)
            }
        }
    }

    @Test
    fun `path test`() {
        println(getTestOutputDir())
    }
}