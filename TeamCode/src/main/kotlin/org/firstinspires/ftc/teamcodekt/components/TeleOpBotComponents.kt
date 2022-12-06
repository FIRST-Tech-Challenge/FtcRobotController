package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.HardwareMap

fun createTeleOpBotComponents(hardwareMap: HardwareMap) =
    TeleOpBotComponents(
        DriveMotors(hardwareMap),
        Claw(hardwareMap),
        Intake(hardwareMap),
        Arm(hardwareMap),
        Wrist(hardwareMap),
        Lift(hardwareMap, VoltageScaler(hardwareMap))
    )

data class TeleOpBotComponents(
    val driveMotors: DriveMotors,
    val claw: Claw,
    val intake: Intake,
    val arm: Arm,
    val wrist: Wrist,
    val lift: Lift,
)
