package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class LiftPID extends LinearOpMode {

    PIDController controller;

    public static double p=0, i=0, d=0;
    public static double f=0;

    public static int target =0;

    DcMotorEx linearSlideMotor, frontSlide, slideOtherer;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linearSlideMotor = hardwareMap.get(DcMotorEx.class,"linearSlide");
        frontSlide = hardwareMap.get(DcMotorEx.class,"frontSlide");
        slideOtherer = hardwareMap.get(DcMotorEx.class, "slideOtherer");

        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p, i, d);
            int liftPos = linearSlideMotor.getCurrentPosition();
            double pid = controller.calculate(liftPos, target);

//            double ff = Math.cos(Math.toRadians(target/ticks_in_degree - 30))*f;

            double power = pid +f;

            linearSlideMotor.setPower(power);
            frontSlide.setPower(power);
            slideOtherer.setPower(power);

            telemetry.addData("slide", linearSlideMotor.getCurrentPosition());
            telemetry.addData("front", frontSlide.getCurrentPosition());
            telemetry.addData("other", slideOtherer.getCurrentPosition());

            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
