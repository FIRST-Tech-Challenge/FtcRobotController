package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.helpers.getMotor
import org.firstinspires.ftc.teamcode.helpers.getServo

//@TeleOp(name = "Roomba Final TeleOp", group = "Center Stage", )
class RoombaCenterStageFinalTeleOp : LinearOpMode() {
    val leftWheel by lazy { hardwareMap.getMotor("left_motor") }
    val rightWheel by lazy { hardwareMap.getMotor("right_motor") }
    val arm by lazy { hardwareMap.getMotor("arm") }
    val wrist by lazy { hardwareMap.getServo("wrist") }
    val gripper by lazy { hardwareMap.getServo("gripper") }
    val airplane by lazy { hardwareMap.getServo("airplane") }
    val maximumPower: Double = 0.5

    override fun runOpMode() {
        leftWheel.mode = RUN_WITHOUT_ENCODER
        rightWheel.mode = RUN_WITHOUT_ENCODER
        rightWheel.direction = FORWARD
        leftWheel.direction = REVERSE
        arm.mode = RUN_USING_ENCODER
        leftWheel.power = 0.0
        rightWheel.power = 0.0
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Left Wheel Direction", leftWheel.direction)
        telemetry.addData("Right Wheel Direction", rightWheel.direction)
        telemetry.addData("arm position", arm.currentPosition)
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            arm.targetPosition = 0
            val leftMotor: Float = -gamepad1.left_stick_y
            val rightMotor: Float = -gamepad1.right_stick_y
            val armUpButton: Boolean = gamepad1.y
            val armDownButton: Boolean = gamepad1.a
            val wristUpButton: Boolean = gamepad2.x
            val wristDownButton: Boolean = gamepad2.b
            val gripperClose: Boolean = gamepad2.right_bumper
            val gripperOpen: Boolean = gamepad2.left_bumper
            val airplane2: Boolean = gamepad2.back
            val resetAirplane: Boolean = gamepad2.start
            leftWheel.power = leftMotor.toDouble()
            rightWheel.power = rightMotor.toDouble()
            telemetry.addData("left_motor_power", leftWheel.power)
            telemetry.addData("right_motor_power", rightWheel.power)
            if (armUpButton == true) {
                arm.direction = REVERSE
                arm.power = maximumPower
            } else if (armDownButton == true) {
                arm.direction = FORWARD
                arm.power = maximumPower
            } else {
                arm.power = 0.0
            }
            if (wristUpButton == true) {
                wrist.direction = Servo.Direction.REVERSE
                wrist.position = 0.8
            } else if (wristDownButton == true) {
                wrist.direction = Servo.Direction.FORWARD
                wrist.position = 0.5
            }
            if (gripperClose == true) {
                gripper.direction = Servo.Direction.FORWARD
                gripper.position = 2.0
            }
            if (gripperOpen == true) {
                gripper.direction = Servo.Direction.REVERSE
                gripper.position = 1.0
            }
            telemetry.addData("wrist position", wrist.position)
            if (airplane2 == true) {
                launchAirplane()
            } else if (resetAirplane == true) {
                airplane.position = 0.0
            }
            telemetry.addData("arm position", arm.currentPosition)
            telemetry.update()
        }
    }

    private fun launchAirplane() {
        airplane.direction = Servo.Direction.FORWARD
        airplane.position = 1.0
        telemetry.addData("airplane", "Lanunching")
    }
}
