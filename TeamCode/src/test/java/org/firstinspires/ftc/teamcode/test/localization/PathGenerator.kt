package org.firstinspires.ftc.teamcode.test.localization

import org.firstinspires.ftc.teamcode.mmooover.kinematics.CommandSerializer
import org.firstinspires.ftc.teamcode.mmooover.kinematics.PathHelper
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

    private fun PathHelper.scoreHighBasket() {
        line(8, 19, (-45).deg)
        run("scoreHighBasket")
    }

    @Test
    fun `generate blue right auto v1`() {
        val commands = path {
            scoreHighBasket()
            line(28, 12, (-180).deg)
            run("intake")
            scoreHighBasket()
            line(28, 22, (-180).deg)
            run("intake")
            scoreHighBasket()
            m(8, 19, (-45).deg)
            m(65, 12, 90.deg)
            m(65, -8, 90.deg)
            run("park")
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