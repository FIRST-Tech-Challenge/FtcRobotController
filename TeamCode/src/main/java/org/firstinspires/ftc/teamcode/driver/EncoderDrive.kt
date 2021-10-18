package org.firstinspires.ftc.teamcode.driver

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.R
import org.firstinspires.ftc.teamcode.robot.Robot

class EncoderDrive(val robot: Robot){

    private val runtime = ElapsedTime()

    val COUNTS_PER_MOTOR_REV = 1440.0

    val DRIVE_GEAR_REDUCTION = 2.0

    val WHEEL_DIAMETER_INCHES = 4.0

    val COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION /
            (WHEEL_DIAMETER_INCHES * hardwareMap.appContext.resources.getString(R.string.pi).toInt())

    var stop = false

    fun encoderDrive(
            speed: Double,
            leftInches: Double,
            rightInches: Double,
            lateralInches: Double,
            timeout: Double,
    ) {
        stop = false
        val newLeftTarget: Int
        val newRightTarget: Int
        val newCenterTarget: Int

        val leftDrive = robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.leftDriveID)) as DcMotor
        val rightDrive = robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.rightDriveID)) as DcMotor
        val centerDrive = robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.centerDriveID)) as DcMotor


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.currentPosition + (leftInches * COUNTS_PER_INCH).toInt()
        newRightTarget = rightDrive.currentPosition + (rightInches * COUNTS_PER_INCH).toInt()
        newCenterTarget = centerDrive.currentPosition + (lateralInches * COUNTS_PER_INCH).toInt()
        leftDrive.targetPosition = newLeftTarget
        rightDrive.targetPosition = newRightTarget
        centerDrive.targetPosition = newCenterTarget

        // Turn On RUN_TO_POSITION
        leftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        centerDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        // reset the timeout time and start motion.
        runtime.reset()
        leftDrive.power = Math.abs(speed)
        rightDrive.power = Math.abs(speed)
        centerDrive.power = Math.abs(speed)

        while (
            !this.stop &&
            runtime.seconds() < timeout &&
            (leftDrive.isBusy() || rightDrive.isBusy() || centerDrive.isBusy())
        ) Thread.sleep(20)


        // Stop all motion;
        leftDrive.power = 0.0
        rightDrive.power = 0.0
        centerDrive.power = 0.0

        // Turn off RUN_TO_POSITION
        leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        centerDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun stop() {
        this.stop = true
    }
}