package org.firstinspires.ftc.teamcode.Debugging.LowLevel;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.DepositSlides;

@Config
@TeleOp
public class DepositSlideTestingOpMode extends OpMode {
    private Hardware hardware = new Hardware();
    private DepositSlides slides;

    public static double
            p = DepositConstants.sp,
            i = DepositConstants.si,
            d = DepositConstants.sd,
            f = DepositConstants.sf;

    public static double targetCM = 0, currentCM = 0;

    @Override
    public void init() {
        hardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides = new DepositSlides(hardware);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        slides.update();

        slides.setPID(p,i, d, f);
        slides.setTargetCM(targetCM);

        slides.command();

        telemetry.addData("Current Position: ", slides.getPosition());
        telemetry.addData("Target Position: ", slides.getTarget());
        telemetry.addData("Ranged Target Position: ", slides.getRangedTarget());
        telemetry.addData("Total Current: ", slides.getCurrent());
        telemetry.addData("Power: ", slides.getPower());
        telemetry.update();

    }

}
