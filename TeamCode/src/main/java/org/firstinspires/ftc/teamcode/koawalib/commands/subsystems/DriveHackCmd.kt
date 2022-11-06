package org.firstinspires.ftc.teamcode.koawalib.commands.subsystems

import com.asiankoala.koawalib.command.commands.MecanumCmd
import com.asiankoala.koawalib.gamepad.functionality.Stick
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import com.asiankoala.koawalib.util.Alliance
import org.firstinspires.ftc.teamcode.koawalib.DriveHack

class DriveHackCmd(
    drive: KMecanumOdoDrive,
    leftStick: Stick,
    rightStick: Stick,
    private val spacegliderToggle: () -> Boolean,
    private val aimbotToggle: () -> Boolean,
    alliance: Alliance = Alliance.BLUE,
) : MecanumCmd(
    drive,
    leftStick,
    rightStick,
    scalars.first,
    scalars.second,
    scalars.third,
    cubics.first,
    cubics.second,
    cubics.third,
    alliance,
    isTranslationFieldCentric = true,
    isHeadingFieldCentric = false,
    { drive.pose.heading },
    hs
) {
    private val driveHack = DriveHack(
        drive::pose,
        R,
        N,
        hs
    )

    override fun processPowers(): Pose {
        val default = super.processPowers()
//        if(spacegliderToggle.invoke()) {
//            driveHack.spaceglide(leftStick.vector)
//        }
//        val v = if (spacegliderToggle.invoke()) {
//            driveHack.spaceglide(default.vec).rotate(-heading.invoke()
//                    + if (alliance == Alliance.RED) 180.0.radians else 0.0)
//        } else default.vec
//
//        val h = if (aimbotToggle.invoke()) {
//            driveHack.aimbot(default.vec)
//        } else default.heading

//        return Pose(v, h)

//        return default
        return default
    }

    companion object {
        private val scalars = Triple(1.0, 1.0, 1.0)
        private val cubics = Triple(1.0, 1.0, 1.0)
        private const val R = 2.0
        private const val N = 1.0
        private val hs = 60.0.radians
    }
}