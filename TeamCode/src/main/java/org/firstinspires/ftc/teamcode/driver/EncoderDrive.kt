package org.firstinspires.ftc.teamcode.driver

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.R
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain
import kotlin.math.abs

class EncoderDrive(val robot: Robot) {

    private val runtime = ElapsedTime()

    var WHEEL_DIAMETER_INCHES: Double = 3.0

    var DRIVE_GEAR_REDUCTION: Double = 20.0

    var COUNTS_PER_MOTOR_REV: Double = 1440.0

    private var COUNTS_PER_INCH: Double = 0.0
        get() = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION /
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

        /*
        val leftDrive =
                robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.leftDriveID)) as DcMotor
        val rightDrive =
                robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.rightDriveID)) as DcMotor
        val centerDrive =
                robot.getRobotPart(hardwareMap.appContext.resources.getInteger(R.integer.centerDriveID)) as DcMotor
         */
        val driveTrain = robot.getRobotPart(DriveTrain::class.java) as DriveTrain
        val leftDrive = robot.hardwareMap.get(hardwareMap.appContext.resources.getStringArray(R.array.LEFT_DRIVE)[0]) as DcMotor
        val rightDrive = robot.hardwareMap.get(hardwareMap.appContext.resources.getStringArray(R.array.RIGHT_DRIVE)[0]) as DcMotor
        val centerDrive = robot.hardwareMap.get(hardwareMap.appContext.resources.getStringArray(R.array.CENTER_DRIVE)[0]) as DcMotor

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
        leftDrive.power = abs(speed)
        rightDrive.power = abs(speed)
        centerDrive.power = abs(speed)

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