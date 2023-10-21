package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

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

    override fun start() {
        super.start()
        shared.intake.raise()
    }

    /** true for lowered, false for raised */
    var lastIntakeStatus = false

    override fun loop() {

        // TODO: test driver relative, check servos

        val gyroRotation = shared.imu.robotYawPitchRollAngles
        val gyroYaw = -gyroRotation.getYaw(AngleUnit.RADIANS)

        val inputTheta = atan2(gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble())
        val driveTheta = inputTheta - gyroYaw
        val inputPower = sqrt((gamepad1.left_stick_y.toDouble() * gamepad1.left_stick_y.toDouble()) + (gamepad1.left_stick_x.toDouble() * gamepad1.left_stick_x.toDouble()))
        val driveRelativeX = cos(driveTheta) * inputPower
        val driveRelativeY = sin(driveTheta) * inputPower

        // Most values are [-1.0, 1.0]

        val control = object {
            val movement = Vector2d(driveRelativeX, driveRelativeY)
            val rotation = gamepad1.right_stick_x.toDouble()
            val intake = gamepad1.a
        }

        // NOTE: This code is(?) bot-relative

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
            leftFrontPower  /=  powerMax
            rightFrontPower /=  powerMax
            leftBackPower   /=  powerMax
            rightBackPower  /=  powerMax
        }

        // Send calculated power to wheels
        shared.motorLeftFront.power =   leftFrontPower
        shared.motorRightFront.power =  rightFrontPower
        shared.motorLeftBack.power =    leftBackPower
        shared.motorRightBack.power =   rightBackPower

        telemetry.addLine("Gyro Yaw: " + gyroRotation.getYaw(AngleUnit.DEGREES))
        telemetry.update()

        // Intake controls
        if (control.intake != lastIntakeStatus) {
            lastIntakeStatus = if (control.intake) {
                shared.intake.lower()
                true
            } else {
                shared.intake.raise()
                false
            }
        }
        shared.intake.active = control.intake
    }
}