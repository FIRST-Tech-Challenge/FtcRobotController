package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Intake;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.IntakeSlides;

@Config
@TeleOp
public class IntakeSlideOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private IntakeSlides slides;
    private Logger logger;
    private GamepadEx controller;

    public static double
            p = IntakeConstants.sp,
            i = IntakeConstants.si,
            d = IntakeConstants.sd;

    public static double targetCM = 0;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        slides = new IntakeSlides(hardware, logger);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        slides.update();

        slides.setPID(p,i, d);
        slides.setTargetCM(targetCM);

        slides.command();

        slides.log();
        logger.print();

    }

}
