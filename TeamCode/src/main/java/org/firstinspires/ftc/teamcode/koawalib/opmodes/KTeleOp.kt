package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.command.commands.ChooseCmd
import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.MecanumCmd
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.math.radians
import com.asiankoala.koawalib.subsystem.odometry.Odometry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.RobotState
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.DepositSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.IntakeSequence
import org.firstinspires.ftc.teamcode.koawalib.commands.sequences.ReadySequence

@TeleOp
class KTeleOp : KOpMode(photonEnabled = true) {
    private val robot by lazy { Robot(Odometry.lastPose) }

    override fun mInit() {
        bindDrive()
        bindStrat()
        bindCycles()
    }

    private fun bindDrive() {
            robot.drive.defaultCommand = MecanumCmd(
                robot.drive,
                driver.leftStick,
                driver.rightStick,
                0.5,
                0.5,
                0.5,
                1.0,
                1.0,
                1.0,
                RobotState.alliance,
                isTranslationFieldCentric = true,
                isHeadingFieldCentric = true,
                { robot.drive.pose.heading },
                60.0.radians
            )

            driver.leftTrigger.onToggle(
                InstantCmd({robot.driveHack.aimbot(driver.leftStick::vector)})
            )

            driver.a.onToggle(
            InstantCmd({robot.driveHack.spaceglide(driver.leftStick::vector)})
        )
    }

    private fun bindStrat() {
        driver.leftBumper.onPress(InstantCmd(RobotState::incStrat))
        driver.rightBumper.onPress(InstantCmd(RobotState::decStrat))
    }

    private fun bindCycles() {
        driver.rightTrigger.onPress(
            ChooseCmd(
                IntakeSequence(robot.claw)
                    .andThen(ReadySequence(robot)),
                DepositSequence(robot)
            ) { RobotState.state == RobotState.State.INTAKING }
        )
    }

    override fun mLoop() {
        Logger.addTelemetryData("state", RobotState.state)
        Logger.addTelemetryData("strat", RobotState.strategy)
        Logger.addTelemetryData("aimbot", driver.a.isToggled)
        Logger.addTelemetryData("spaceglide", driver.leftTrigger.isToggled)
    }
}
