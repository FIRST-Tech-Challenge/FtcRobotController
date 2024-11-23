package org.firstinspires.ftc.teamcode.FancyTeleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class HobbesZeroer extends OpMode {

    Hobbes hob = null;
    public static double extendo, extendoArm, extendoWrist, slidesArm, slidesWrist, intake, clawPos;
    public static int slides;
    public static boolean extendoON = false;
    public static boolean extendoArmWristON = false;
    public static boolean slidesArmWristON = false;
    public static boolean intakeON = false;
    public static boolean slidesON = false;
    public static boolean clawON = false;
    Telemetry tele;

    @Override
    public void init() {
        tele = FtcDashboard.getInstance().getTelemetry();
        hob = new Hobbes();
        hob.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (extendoON) hob.servosController.setExtendo(extendo);
        if (extendoArmWristON) hob.servosController.setExtendoArmWrist(extendoArm, extendoWrist);
        if (slidesArmWristON) hob.servosController.setSlidesArmWrist(slidesArm, slidesWrist);
        if (slidesON) hob.slidesController.setTarget(slides);
        if (intakeON) hob.servosController.spintake(intake);
        if (clawON) hob.servosController.setClawPrecise(clawPos);
        hob.tick();
    }
}
