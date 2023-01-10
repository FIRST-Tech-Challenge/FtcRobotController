package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.hardware.rev.RevColorSensorV3
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

abstract class BaseBotComponents {
    abstract val claw: Claw
    abstract val intake: Intake
    abstract val arm: Arm
    abstract val wrist: Wrist
    abstract val lift: Lift

    fun updateBaseComponents() {
        claw.update()
        arm.update()
        wrist.update()
        lift.update()
    }
}

fun createTeleOpBotComponents(): TeleOpBotComponents {
    return TeleOpBotComponents(
        hwMap(DeviceNames.COLOR_SENSOR),
        Drivetrain(),
        Claw(),
        Intake(),
        Arm(),
        Wrist(),
        Lift()
    )
}

data class TeleOpBotComponents(
    val rcs: RevColorSensorV3,
    val drivetrain: Drivetrain,
    override val claw: Claw,
    override val intake: Intake,
    override val arm: Arm,
    override val wrist: Wrist,
    override val lift: Lift,
) : BaseBotComponents()

fun createAutoBotComponents() = AutoBotComponents(
    SampleMecanumDrive(hwMap),
    Claw(),
    Intake(),
    Arm(),
    Wrist(),
    Lift(),
)

data class AutoBotComponents(
    val drive: SampleMecanumDrive,
    override val claw: Claw,
    override val intake: Intake,
    override val arm: Arm,
    override val wrist: Wrist,
    override val lift: Lift,
) : BaseBotComponents()
