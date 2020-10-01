package org.firstinspires.ftc.teamcode.baseClasses

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.teamcode.baseClasses.navigation.AutonomousTarget
import org.firstinspires.ftc.teamcode.baseClasses.navigation.VuforiaNavigation

abstract class MecanumAutoOpMode: BaseLinearOpMode() {
    private val navigation = VuforiaNavigation(::logger)

    abstract var autonomousSide: AutonomousSide

    /**
     * The main action loop
     */
    @Throws(InterruptedException::class)
    abstract fun mecaLoop()


    fun driveToTarget(target: AutonomousTarget) {
        val targetMatrix = target.positionMatrix(autonomousSide)
    }


    override fun runLoop() {
        // Initialize the navigation. Hold either the left or right trigger on gamepad 1 when you press the init
        // button to setup Vuforia in debug mode where it will log all values
        //FIXME: For now always logging since I don't have a gamepad, remove the `|| true` before competition
        navigation.init(hardwareMap, logging = gamepad1.right_bumper || gamepad1.left_bumper || true)
        waitForStart()

        while (opModeIsActive()) {
            mecaLoop()
        }

        navigation.deinit()
    }


    enum class AutonomousSide {
        RED, BLUE
    }
}