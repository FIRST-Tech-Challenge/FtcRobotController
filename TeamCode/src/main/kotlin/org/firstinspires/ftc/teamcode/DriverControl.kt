package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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

@TeleOp(name = "Kotlin Driver Control", group = "Kt")
class DriverControl: OpMode() {

    private lateinit var shared: BotShared

    override fun init() {
//        val setter = DriverControl::tagCamera.setter
        shared = BotShared(this)
        shared.drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
    }

    override fun loop() {

        // Most values are [-1.0, 1.0]
        val control = object {
            val movement = Vector2d(gamepad1.left_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())
            val rotation = gamepad1.right_stick_x.toDouble();
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

        // Send calculated power to wheels
        shared.motorFrontLeft.power =   leftFrontPower
        shared.motorFrontRight.power =  rightFrontPower
        shared.motorBackLeft.power =    leftBackPower
        shared.motorBackRight.power =   rightBackPower
    }
}