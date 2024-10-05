package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.config.CDConfig
import org.firstinspires.ftc.teamcode.drive.CDMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

abstract class OpModeBase : CommandOpMode() {
    lateinit var hardware: HardwareManager
    lateinit var mecanumDrive: CDMecanumDrive
    lateinit var driverGamepad: GamepadEx
    lateinit var accessoryGamepad: GamepadEx
    lateinit var multiTelemetry: MultipleTelemetry

    fun initHardware() {
        hardware = HardwareManager(CDConfig(), hardwareMap)
        mecanumDrive = CDMecanumDrive(hardware)
        multiTelemetry = MultipleTelemetry(telemetry)

        // Subsystems
//        deliverySubsystem = try { DeliverySubsystem(hardware, multiTelemetry) } catch (e: Exception) { null }
//        droneSubsystem = try { DroneSubsystem(hardware, multiTelemetry) } catch (e: Exception) { null }
//        intakeSubsystem = try { IntakeSubsystem(hardware, multiTelemetry) } catch (e: Exception) { null }
//        suspendSubsystem = try { SuspendSubsystem(hardware, multiTelemetry) } catch (e: Exception) { null }
//
//        val subsystems = listOf<Subsystem?>(
//            deliverySubsystem,
//            droneSubsystem,
//            intakeSubsystem,
//            suspendSubsystem
//        )
//
//        register(*subsystems.filterNotNull().toTypedArray())

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