package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

abstract class BaseBotComponents {
    abstract val claw: Claw
    abstract val intake: Intake
    abstract val arm: Arm
    abstract val wrist: Wrist
    abstract val lift: Lift

    fun updateComponents() {
        claw.update()
        arm.update()
        wrist.update()
    }
}

fun createTeleOpBotComponents(hwMap: HardwareMap, voltageScaler: VoltageScaler) =
    TeleOpBotComponents(
        Drivetrain(hwMap),
        Claw(hwMap),
        Intake(hwMap),
        Arm(hwMap),
        Wrist(hwMap),
        Lift(hwMap, voltageScaler)
    )

data class TeleOpBotComponents(
    val drivetrain: Drivetrain,
    override val claw: Claw,
    override val intake: Intake,
    override val arm: Arm,
    override val wrist: Wrist,
    override val lift: Lift,
) : BaseBotComponents()

fun createAutoBotComponents(hwMap: HardwareMap, voltageScaler: VoltageScaler) =
    AutoBotComponents(
        SampleMecanumDrive(hwMap),
        Claw(hwMap),
        Intake(hwMap),
        Arm(hwMap),
        Wrist(hwMap),
        Lift(hwMap, voltageScaler)
    )

data class AutoBotComponents(
    val drive: SampleMecanumDrive,
    override val claw: Claw,
    override val intake: Intake,
    override val arm: Arm,
    override val wrist: Wrist,
    override val lift: Lift,
) : BaseBotComponents()
