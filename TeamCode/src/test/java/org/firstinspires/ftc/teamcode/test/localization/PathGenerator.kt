package org.firstinspires.ftc.teamcode.test.localization

import org.firstinspires.ftc.teamcode.mmooover.kinematics.CommandSerializer
import org.firstinspires.ftc.teamcode.mmooover.kinematics.path
import org.junit.jupiter.api.Test
import java.io.DataOutputStream
import java.io.File


class PathGenerator {
    private fun getTestOutputDir(): File {
        val path = File(System.getProperty("user.dir")!!).resolve("build").resolve("outputs")
            .resolve("paths")
        path.mkdirs()
        return path
    }

    @Test
    fun `generate blue right auto v1`() {
        val commands = path {
            autoInsertHolds = true
            m(0, 0, 0.deg)
            m(0, 48)
            m(48, 48, 90.deg)
            run("ScoreHigh")
            // automatic insertion happens here
            m(48, 24, 180.deg)
        }.generate2()

        val outFile = getTestOutputDir().resolve("AutoBlueRight1.bin")
        if (outFile.exists()) outFile.delete()
        outFile.outputStream().use {
            DataOutputStream(it).use { out ->
                CommandSerializer.serialize(commands, out)
            }
        }
    }

    @Test
    fun `path test`() {
        println(getTestOutputDir())
    }
}