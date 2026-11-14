package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.DriveMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.IntakeShootMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RampMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils.StaticStore
import kotlin.compareTo
import kotlin.time.Duration.Companion.seconds

@TeleOp
class TeleOp: LinearOpMode() {
    lateinit var hw: GerthaHardwareMap

    override fun runOpMode() {
        hw = GerthaHardwareMap(hardwareMap)
        val driveMachine = DriveMachine(hw)
        val robotState = RobotState(hw)
        val gamepadState = GamepadState(gamepad1, gamepad2)
        val intakeShootMachine = IntakeShootMachine(hw)
        val rampMachine = RampMachine(hw)

        if (StaticStore.duration() > 30.seconds) {
            hw.pinpoint.resetPosAndIMU()
        }

        waitForStart()

        driveMachine.init()
        intakeShootMachine.init()
        rampMachine.init()

        while (opModeIsActive()) {

            robotState.update()
            gamepadState.update()
            driveMachine.update(robotState, gamepadState)
            intakeShootMachine.update(robotState, gamepadState)
            rampMachine.update(gamepadState)



            telemetry.addData("ppAngle", robotState.ppThetaDeg)
            telemetry.addData("count", intakeShootMachine.count)
            telemetry.update()
        }

        StaticStore.mark()

    }
}