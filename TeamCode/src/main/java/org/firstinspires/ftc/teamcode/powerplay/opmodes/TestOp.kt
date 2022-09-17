package org.firstinspires.ftc.teamcode.powerplay.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.powerplay.Robot
import org.firstinspires.ftc.teamcode.util.GamepadUtil.left_trigger_pressed
import org.firstinspires.ftc.teamcode.util.GamepadUtil.right_trigger_pressed
import org.firstinspires.ftc.teamcode.util.math.MathUtil

@TeleOp
class TestOp: OpMode() {
    private val robot = Robot()

    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    private fun driveControl() {
        drive = MathUtil.cubicScaling(0.75, -gamepad1.left_stick_y.toDouble()) * 0.85
        strafe = MathUtil.cubicScaling(0.75, gamepad1.left_stick_x.toDouble()) * 0.85
        rotate = gamepad1.right_stick_x.toDouble() * 0.65
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }

    private fun intakeControl() {
        if (gamepad1.right_trigger_pressed) {
            robot.reverseIntake()
        }

        if (!gamepad1.right_trigger_pressed && !gamepad1.left_trigger_pressed) {
            robot.stopIntake()
        }

        if (gamepad1.left_trigger_pressed && !robot.intakeSequence.running && robot.isConeIn()) {
            robot.intakeSequence.start()
        }

        if (!gamepad1.left_trigger_pressed && robot.intakeSequence.running) {
            robot.intakeSequence.stop()
            robot.intakeSequence.reset()
        }

        if (robot.intakeSequence.running && gamepad1.left_trigger_pressed) {
            robot.intakeSequence.update()
        }
    }

    private fun getTelemetry() {
        telemetry.addData("dSensor", robot.intake.dSensor.getDistance(DistanceUnit.MM))
    }

    override fun init() {
        robot.init(hardwareMap)
        robot.reset()
    }

    override fun loop() {
        driveControl()
        intakeControl()
        getTelemetry()
        robot.update()
    }
}