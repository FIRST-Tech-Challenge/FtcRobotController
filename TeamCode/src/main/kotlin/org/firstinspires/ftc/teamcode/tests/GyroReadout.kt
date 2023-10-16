package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

@TeleOp(name = "Gyroscope Readout", group = "KtTest")
class GyroReadout : OpMode() {

    lateinit var imu: IMU
    /**
     * Code to run ONCE when the driver hits INIT
     */
    override fun init() {
        imu = hardwareMap[IMU::class.java, "imu"]

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized")
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    override fun start() {
        // IMU orientation/calibration
        val logo = RevHubOrientationOnRobot.LogoFacingDirection.UP
        val usb = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logo, usb)
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    override fun loop() {
        val gyroRotation = imu.robotYawPitchRollAngles
        // YAW INCREASES COUNTERCLOCKWISE!!!
        val gyroYaw = gyroRotation.getYaw(AngleUnit.DEGREES);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Yaw: %2.f degrees", gyroYaw)
        telemetry.update()
    }
}