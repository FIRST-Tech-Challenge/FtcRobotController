package org.firstinspires.ftc.teamcode.testing.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.MecanumCmd
import com.asiankoala.koawalib.subsystem.odometry.Odometry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.testing.Robot
import org.firstinspires.ftc.teamcode.testing.subsystems.Lift

@TeleOp
class RobotOp : KOpMode(photonEnabled = true) {
    private val robot by lazy { Robot(Odometry.lastPose) }

        override fun mInit() {
            robot.drive.defaultCommand = MecanumCmd(
                robot.drive,
                driver.leftStick,
                driver.rightStick,
                0.9,
                0.9,
                0.9,
                1.0,
                1.0,
                1.0
            )

            driver.dpadUp.onPress(Lift.LiftMove(0.7, robot.lift))
            driver.dpadDown.onPress(Lift.LiftMove(-0.4, robot.lift))
            driver.dpadUp.onRelease(Lift.LiftMove(0.0, robot.lift))
            driver.dpadDown.onRelease(Lift.LiftMove(0.0, robot.lift))
    }
}