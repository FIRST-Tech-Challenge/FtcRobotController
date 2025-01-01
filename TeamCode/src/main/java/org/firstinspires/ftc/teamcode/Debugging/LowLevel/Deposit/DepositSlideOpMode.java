package org.firstinspires.ftc.teamcode.Debugging.LowLevel.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;
import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.DepositSlides;

@Config
@TeleOp
public class DepositSlideOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private DepositSlides slides;
    private Logger logger;
    private GamepadEx controller;

    public static double
            p = DepositConstants.sp,
            i = DepositConstants.si,
            d = DepositConstants.sd,
            f = DepositConstants.sf;

    public static double targetCM = 0;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        controller = new GamepadEx(gamepad1);
        logger = new Logger(telemetry, controller);
        slides = new DepositSlides(hardware, logger);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        controller.readButtons();

        slides.update();

        slides.setPID(p,i, d, f);
        slides.setTargetCM(targetCM);

        slides.command();

        slides.log();
        logger.print();
    }

    private void callI2C() {
        hardware.pinPoint.update();
        hardware.intakeCS.updateColors();
    }

}
