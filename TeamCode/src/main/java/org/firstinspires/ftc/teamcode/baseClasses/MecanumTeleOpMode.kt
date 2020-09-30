package org.firstinspires.ftc.teamcode.baseClasses

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import kotlin.math.cos
import kotlin.math.sin

/// Credits for the mecanum drive portion of this class go to FTC11848
/// https://www.reddit.com/r/FTC/comments/9ou0ib/mecanum_drive_code/e7wrp9f?utm_source=share&utm_medium=web2x&context=3

abstract class MecanumTeleOpMode: BaseLinearOpMode() {
    /**
     * Drive the robot according to the gamepad
     */
    private fun mecaDrive() {
        val params = MecanumDriveParameters.fromGamepad(gamepad1, hardware = hardware)
        val instructions = PowerInstructions.fromDriveParams(params)
        instructions.applyOnHardware(hardware)
    }


    /**
     * The action executed before the motion of the robot
     */
    open fun preMoveLoop() { }

    /**
     * The action executed after the motion of the robot
     */
    @Throws(InterruptedException::class)
    abstract fun mecaLoop()

    override fun runLoop() {
        waitForStart()

        while (opModeIsActive()) {
            preMoveLoop()
            mecaDrive()
            mecaLoop()
        }
    }
}