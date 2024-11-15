package org.firstinspires.ftc.teamcode

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drivetrain.FeedForward
import org.firstinspires.ftc.teamcode.helpers.MecanumDrive
import org.firstinspires.ftc.teamcode.helpers.PowerVector
import org.firstinspires.ftc.teamcode.helpers.bulkCachingMode
import org.firstinspires.ftc.teamcode.helpers.getMotor
import org.firstinspires.ftc.teamcode.helpers.getServo
import org.firstinspires.ftc.teamcode.helpers.leftBackMotor
import org.firstinspires.ftc.teamcode.helpers.leftFrontMotor
import org.firstinspires.ftc.teamcode.helpers.mecanumDrive
import org.firstinspires.ftc.teamcode.helpers.rightBackMotor
import org.firstinspires.ftc.teamcode.helpers.rightFrontMotor

//@Config
@TeleOp(name = "Testing TeleOp", group = "Into the Deep")
class TestingTeleop : OpMode() {

    val mecanumDrive by lazy { hardwareMap.mecanumDrive() }
    val leftFront by lazy { hardwareMap.leftFrontMotor() }
    val rightFront by lazy { hardwareMap.rightFrontMotor() }
    val leftBack by lazy { hardwareMap.leftBackMotor() }
    val rightBack by lazy { hardwareMap.rightBackMotor() }
    val arm by lazy { hardwareMap.getMotor("arm_motor") }
    val extension by lazy { hardwareMap.getMotor("extension_motor") }
    val pincer by lazy { hardwareMap.getServo("pincer") }

    companion object {
        @JvmField var armPosition = 0
        @JvmField var armGearRatio = 20
        @JvmField var armPositionModifier = 1
        @JvmField var maximumArmPosition = 700
        @JvmField var minimumArmPosition = 0
        @JvmField var armExtension = 0
        @JvmField var maximumArmExtension = 600
        @JvmField var minimumArmExtension = 0
        @JvmField var extensionRate = 100
    }

    override fun init() {
        hardwareMap.bulkCachingMode = BulkCachingMode.AUTO

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry);

        mecanumDrive.zeroEncoders()
        mecanumDrive.telemetry = telemetry
        arm.zeroPowerBehavior = BRAKE
        arm.mode = STOP_AND_RESET_ENCODER
        arm.targetPosition = arm.currentPosition
        arm.direction = FORWARD
        arm.velocity = 1000.0

        extension.mode = STOP_AND_RESET_ENCODER
        extension.zeroPowerBehavior = BRAKE
        extension.targetPosition = extension.currentPosition
        extension.direction = REVERSE
        extension.velocity = 1000.0

        arm.mode = RUN_TO_POSITION
        extension.mode = RUN_TO_POSITION
        arm.power = 1.0
        extension.power = 1.0
    }

    override fun loop() {
        armControl(gamepad = gamepad1, arm = arm, extension = extension, telemetry = telemetry)

        drivetrain(gamepad = gamepad1, mecanumDrive = mecanumDrive)
    }


    fun armControl(gamepad: Gamepad, arm: DcMotorEx, extension: DcMotorEx, telemetry: Telemetry) {
        if (gamepad.dpad_up) armPosition -= armGearRatio * armPositionModifier
        if (gamepad.dpad_down) armPosition += armGearRatio * armPositionModifier
        armPosition = clamp(armPosition, minimumArmPosition, maximumArmPosition)

        telemetry.addData("target arm position:", armPosition)
        telemetry.addData("current arm position:", arm.currentPosition)
        arm.targetPosition = armPosition
        arm.velocity = 1000.0

        if (gamepad.dpad_left) armExtension += extensionRate
        if (gamepad.dpad_right) armExtension -= extensionRate
        armExtension = clamp(armExtension, minimumArmExtension, maximumArmExtension)

        telemetry.addData("target arm extension:", armExtension)
        telemetry.addData("current arm extension:", extension.currentPosition)
        extension.targetPosition = armExtension
        extension.velocity = 1000.0

        if (gamepad.right_trigger > 0) pincer.position = 0.0
        if (gamepad.right_bumper) pincer.position = 1.0
    }

    fun drivetrain(gamepad: Gamepad, mecanumDrive: MecanumDrive) {
        mecanumDrive.powerVector = PowerVector(
            axial = -gamepad.left_stick_y.toDouble(),
            lateral = gamepad.left_stick_x.toDouble(),
            yaw = gamepad.right_stick_x.toDouble()
        )
    }
}