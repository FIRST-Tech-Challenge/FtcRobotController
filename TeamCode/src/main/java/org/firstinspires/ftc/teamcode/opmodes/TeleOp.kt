package org.firstinspires.ftc.teamcode.opmodes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.util.RobotHardware
import org.firstinspires.ftc.teamcode.util.TelemetryLogger

lateinit var logger: TelemetryLogger

@TeleOp(name = "TeleOp")
@Suppress("unused")
class TeleOp : OpMode() {
    private lateinit var gamepad: GamepadEx
    private lateinit var hardware: RobotHardware

    override fun init() {
        gamepad = GamepadEx(gamepad1)
        logger = TelemetryLogger(telemetry)

        logger.info("Initialized OpMode")
    }

    override fun start() {
        hardware = RobotHardware(hardwareMap, gamepad)
        logger.info("Started OpMode")
    }

    override fun loop() {
        hardware.driveTrain.drive()
        gamepad.readButtons()
    }
}