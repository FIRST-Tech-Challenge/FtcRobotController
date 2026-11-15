package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.DriveMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.IntakeShootMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RampMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils.StaticStore
import kotlin.time.Duration.Companion.seconds


abstract class TeleOperation(val isRed: Boolean): LinearOpMode() {
    lateinit var hw: GerthaHardwareMap

    override fun runOpMode() {
        hw = GerthaHardwareMap(hardwareMap)
        val driveMachine = DriveMachine(hw)
        val robotState = RobotState(hw, isRed)
        val gamepadState = GamepadState(gamepad1, gamepad2)
        val intakeShootMachine = IntakeShootMachine(hw)
        val rampMachine = RampMachine(hw)

        if (StaticStore.duration() > 30.seconds) {
            hw.pinpoint.resetPosAndIMU()
        }

        waitForStart()

        driveMachine.init(robotState)
        intakeShootMachine.init()
        rampMachine.init()

        while (opModeIsActive()) {

            robotState.update()
            gamepadState.update()
            driveMachine.update(robotState, gamepadState)
            intakeShootMachine.update(robotState, gamepadState)
            rampMachine.update(gamepadState)

            if (gamepadState.start1) {
                hw.pinpoint.setHeading(0.0, AngleUnit.RADIANS);
            }

            if (gamepadState.back2) {
                intakeShootMachine.count = 0
            }



            telemetry.addData("ppAngle", robotState.ppThetaDeg)
            telemetry.addData("count", intakeShootMachine.count)
            telemetry.update()
        }

        StaticStore.mark()

    }
}