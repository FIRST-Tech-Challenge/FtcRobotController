package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.ShooterImpl2
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
        val shooterImpl = ShooterImpl2(hw)
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

            wasa = gamepad1.a
            wasb = gamepad1.b

//            hw.setPollenVelocity(targetVel)
//            hw.pollenShooter.power = 1.0
            shooterImpl.setTarget(targetVel)
            shooterImpl.tickShooter()



            telemetry.addData("targetVel", targetVel)
            telemetry.addData("actualVel", hw.pollenShooter.velocity)
            telemetry.update()
        }
    }

}