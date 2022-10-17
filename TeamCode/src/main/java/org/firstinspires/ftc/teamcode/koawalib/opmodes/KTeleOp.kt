package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.MecanumCmd
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.math.radians
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lights
import org.firstinspires.ftc.teamcode.koawalib.subsystems.SlideMove

@TeleOp
class KTeleOp : KOpMode() {
    private val robot by lazy { Robot(startPose) }
    private val startPose = Pose(-60.0, -10.0, 0.0.radians)

    override fun mInit() {
        robot.drive.setDefaultCommand(
            MecanumCmd(
                robot.drive,
                driver.leftStick,
                driver.rightStick.xInverted,
                1.0,
                1.0,
                1.0,
                0.9,
                0.9,
                0.9
            )
        )

        robot.lightsDevice.setPattern(Lights.BlinkinPattern.RED)

        driver.x.onPress(InstantCmd({ driver.rumbleBlips(3) }))
        driver.y.onPress(InstantCmd({ driver.rumble(2500) }))

//        driver.dpadUp.onPress(InstantCmd({robot.slidesMotor.setPower(0.25)}, robot.slidesMotor))
//        driver.dpadDown.onPress(InstantCmd({robot.slidesMotor.setPower(-0.25)}, robot.slidesMotor))
//        driver.dpadUp.onRelease(InstantCmd({robot.slidesMotor.setPower(0.0)}, robot.slidesMotor))
//        driver.dpadDown.onRelease(InstantCmd({robot.slidesMotor.setPower(0.0)}, robot.slidesMotor))
//
//        driver.a.onPress(Claw.ClawOpen(robot.clawServo))
//        driver.b.onPress(Claw.ClawClose(robot.clawServo))

//        driver.leftBumper.onPress(Arm.ArmReset(robot.armServo))
//        driver.rightBumper.onPress(Arm.ArmOut(robot.armServo))


    }

    override fun mLoop() {
    }
}
