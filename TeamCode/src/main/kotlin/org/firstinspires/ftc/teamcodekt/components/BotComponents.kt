package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

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
    val claw: Claw,
    val intake: Intake,
    val arm: Arm,
    val wrist: Wrist,
    val lift: Lift,
)

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
    val claw: Claw,
    val intake: Intake,
    val arm: Arm,
    val wrist: Wrist,
    val lift: Lift,
)
