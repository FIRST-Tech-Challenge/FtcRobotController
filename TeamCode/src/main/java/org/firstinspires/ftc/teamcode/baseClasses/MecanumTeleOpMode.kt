package org.firstinspires.ftc.teamcode.baseClasses

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import kotlin.math.cos
import kotlin.math.sin


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
