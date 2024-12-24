package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.SampleDetector;

@Config
@TeleOp
public class SampleDetectorOpMode extends OpMode {

    private Hardware hardware = new Hardware();
    private GamepadEx controller;
    private Logger logger;

    private SampleDetector detector;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);

        detector = new SampleDetector(hardware, logger);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();
        detector.update();
        detector.log();
        logger.print();
    }
}
