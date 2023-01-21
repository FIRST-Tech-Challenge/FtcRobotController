package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="Lift PID")
public class LiftPID extends LinearOpMode {

    PIDController liftController, armController;

    public static double p=0.04, i=0, d=0.0001;
    public static double f=0.06;
    public static double p_arm=0.025, i_arm=0.05, d_arm=0.0001;
    public static double f_arm=0.16;

    public static int target =0;
    int armTarget = 300;

    DcMotorEx linearSlideMotor, frontSlide, slideOtherer, armServo;

    @Override
    public void runOpMode() throws InterruptedException {

        liftController = new PIDController(p, i, d);
        armController = new PIDController(p_arm, i_arm, d_arm);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linearSlideMotor = hardwareMap.get(DcMotorEx.class,"linearSlide");
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide = hardwareMap.get(DcMotorEx.class,"frontSlide");
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOtherer = hardwareMap.get(DcMotorEx.class, "slideOtherer");
        slideOtherer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armServo = hardwareMap.get(DcMotorEx.class, "armServo");
        slideOtherer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks_in_degree = 1425/360;

        waitForStart();
        while (opModeIsActive()){
            liftController.setPID(p, i, d);
            int liftPos = linearSlideMotor.getCurrentPosition();
            double pid = liftController.calculate(liftPos, target);

            double power = pid +f;

            linearSlideMotor.setPower(power);
            frontSlide.setPower(power);
            slideOtherer.setPower(power);

            telemetry.addData("slide", linearSlideMotor.getCurrentPosition());
            telemetry.addData("front", frontSlide.getCurrentPosition());
            telemetry.addData("other", slideOtherer.getCurrentPosition());

            int armPos = armServo.getCurrentPosition();
            double pidArm = armController.calculate(armPos, armTarget);

            double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f;

            double armPower = pidArm +ff;
            if (target ==0 && armPos<10){
                armServo.setPower(0);
            } else {
                armServo.setPower(armPower);
            }

            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
