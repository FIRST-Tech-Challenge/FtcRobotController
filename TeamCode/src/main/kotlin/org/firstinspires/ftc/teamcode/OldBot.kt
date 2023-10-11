package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.abs
import kotlin.math.max

/*
    CONTROL IDEAS:

    1. Driver-relative control *by default*
        a. Hold some button down to use bot-relative (for precise movements)
        b. Use the onboard gyroscope for this
    2.
 */

@TeleOp(name = "JV bot", group = "Kt")
class OldBot: OpMode() {

    private lateinit var shared: BotShared

    lateinit var liftMotor: DcMotorEx
    lateinit var leftClaw: Servo
    lateinit var rightClaw: Servo

    override fun init() {
//        val setter = DriverControl::tagCamera.setter
        shared = BotShared(this)
        liftMotor = hardwareMap[DcMotorEx::class.java, "leftLift"]
        leftClaw = hardwareMap[Servo::class.java, "leftClaw"]
        rightClaw = hardwareMap[Servo::class.java, "rightClaw"]
        shared.drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
    }

    fun scrap() {
        if (gamepad1.left_bumper) {
            leftClaw.position = 0.0
            rightClaw.position = 0.0
        } else if (gamepad1.right_bumper) {
            leftClaw.position = 0.25
            rightClaw.position = -0.25
        }

        // up
        if (gamepad1.right_trigger > 0.5) {
            liftMotor.targetPosition = 800
            liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            liftMotor.power = 1.0
        // down
        } else if (gamepad1.left_trigger > 0.5) {
            liftMotor.targetPosition = 0
            liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            liftMotor.power = 0.25
        } else {
            liftMotor.power = 0.1
        }
    }

    override fun loop() {

        // Most values are [-1.0, 1.0]
        val control = object {
            val movement = Vector2d((if (gamepad1.dpad_left) -1.0 else 0.0) + (if (gamepad1.dpad_right) 1.0 else 0.0), (if (gamepad1.dpad_up) -1.0 else 0.0) + (if (gamepad1.dpad_down) 1.0 else 0.0))
            val rotation = gamepad1.right_stick_x.toDouble()
        }

        val gyroRotation = shared.imu.robotYawPitchRollAngles
        // gyroRotation.getYaw(AngleUnit.DEGREES)

        // NOTE: This code is bot-relative

        var powerMax: Double
        // Note: pushing stick forward gives negative value
        // Addie's NOTE: ^ BUT WHY????
        val axial =     control.movement.y
        val lateral =   control.movement.x
        val yaw =       control.rotation

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        var leftFrontPower =    axial + lateral + yaw
        var rightFrontPower =   axial - lateral - yaw
        var leftBackPower =     axial - lateral + yaw
        var rightBackPower =    axial + lateral - yaw


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        // NOTE: is this really "normalization?" I need to rework this.
        powerMax = max(abs(leftFrontPower), abs(rightFrontPower))
        powerMax = max(powerMax, abs(leftBackPower))
        powerMax = max(powerMax, abs(rightBackPower))

        if (powerMax > 1.0) {
            leftFrontPower /=   powerMax
            rightFrontPower /=  powerMax
            leftBackPower /=    powerMax
            rightBackPower /=   powerMax
        }

        shared.motorFrontLeft.power =   leftFrontPower
        shared.motorFrontRight.power =  rightFrontPower
        shared.motorBackLeft.power =    leftBackPower
        shared.motorBackRight.power =   rightBackPower

        // Send calculated power to wheels
        shared.motorFrontLeft.power =   leftFrontPower
        shared.motorFrontRight.power =  rightFrontPower
        shared.motorBackLeft.power =    leftBackPower
        shared.motorBackRight.power =   rightBackPower

        telemetry.addData("left stick: ", "x = %4.2f, \ny = %4.2f", gamepad1.left_stick_x, gamepad1.left_stick_y)
        telemetry.addData("right stick", "x = %4.2f, \ny = %4.2f", gamepad1.right_stick_x, gamepad1.right_stick_y)
        telemetry.update()

        scrap()
    }
}