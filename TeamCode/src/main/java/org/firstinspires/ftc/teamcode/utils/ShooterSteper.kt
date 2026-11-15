package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import org.firstinspires.ftc.teamcode.ShooterImpl
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.IntakeShootMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState

@TeleOp
class ShooterSteper : LinearOpMode() {

    lateinit var hw: GerthaHardwareMap

    override fun runOpMode() {

        var targetVel = 1000.0

        var wasa = false
        var wasb = false

        hw = GerthaHardwareMap(hardwareMap)
        val robotState = RobotState(hw)
        val gamepadState = GamepadState(gamepad1, gamepad2)
        val shooterImpl = ShooterImpl(hw)
        val intakeShootMachine = IntakeShootMachine(hw)

        waitForStart()

        intakeShootMachine.init()

        while(opModeIsActive()){

            gamepadState.update()
            robotState.update()
            intakeShootMachine.update(robotState, gamepadState)

            if (gamepad1.a && !wasa){
                targetVel += 20
            } else if (gamepad1.b && !wasb){
                targetVel -= 20
            }

            if (gamepad2.x) {
                hw.pollenBallStop.position = POLLEN_GO
            }
            else if (gamepad2.y) {
                hw.pollenBallStop.position = POLLEN_STOP
            }

            wasa = gamepad1.a
            wasb = gamepad1.b

//            hw.setPollenVelocity(targetVel)
//            hw.pollenShooter.power = 1.0
            shooterImpl.setTarget(targetVel)
            shooterImpl.tickShooter()


            telemetry.addData("count", intakeShootMachine.count)
            telemetry.addData("power", hw.pollenShooter.power)
            telemetry.addData("targetVel", targetVel)
            telemetry.addData("actualVel", hw.pollenShooter.velocity)
            telemetry.update()
        }
    }

}