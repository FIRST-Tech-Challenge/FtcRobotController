package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.config.CDConfig
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.subsystem.ActiveIntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.GripperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

abstract class OpModeBase : CommandOpMode() {
    lateinit var hardware: HardwareManager
    lateinit var mecanumDrive: CDMecanumDrive
    lateinit var driverGamepad: GamepadEx
    lateinit var accessoryGamepad: GamepadEx
    lateinit var multiTelemetry: MultipleTelemetry

    // Subsystems
    lateinit var activeIntakeSubsystem: ActiveIntakeSubsystem
    lateinit var gripperSubsystem: GripperSubsystem
    lateinit var viperArmSubsystem: ViperArmSubsystem

    fun initHardware() {
        hardware = HardwareManager(CDConfig(), hardwareMap)
        mecanumDrive = CDMecanumDrive(hardware)
        multiTelemetry = MultipleTelemetry(telemetry)

        activeIntakeSubsystem = ActiveIntakeSubsystem(hardware, multiTelemetry)
        gripperSubsystem = GripperSubsystem(hardware, multiTelemetry)
        viperArmSubsystem = ViperArmSubsystem(hardware, multiTelemetry)

        register(
            activeIntakeSubsystem,
            gripperSubsystem,
            viperArmSubsystem
        )

        driverGamepad = GamepadEx(gamepad1)
        accessoryGamepad = GamepadEx(gamepad2)
    }

    enum class Alliance {
        RED, BLUE;

        fun adjust(input: Double): Double {
            return if (this == BLUE) input else -input
        }
    }
}