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
@TeleOp(name="Lift PID", group="pid")
public class LiftPID extends LinearOpMode {

    PIDController liftController;
    ArmPIDController armPIDController;

    public static double p=0.025, i=0, d=0.000001;
    public static double f=0.03;
    public static double p_arm=0.025, i_arm=0.05, d_arm=0.0001;
    public static double f_arm=0.16;
    public static double multiplier = 1;
    public  static double multiplierZero = 0.2;

    public int ticks= 385;
    public static int target =0;
    int armTarget = 300;

    DcMotorEx linearSlideMotor, frontSlide, slideOtherer, armServo;

    @Override
    public void runOpMode() throws InterruptedException {
        double multi = multiplier;

        liftController = new PIDController(p, i, d);

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

        slideOtherer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double ticks_in_degree = 1425/360;

        armPIDController = new ArmPIDController(armServo);

        waitForStart();
        while (opModeIsActive()){

            liftController.setPID(p, i, d);
            int liftPos = linearSlideMotor.getCurrentPosition();
            double pid = liftController.calculate(liftPos, target);

            double power = pid +f;

            if (target == 0 && liftPos<100){
telemetry.addData("power", 0);
                linearSlideMotor.setPower(0);
                frontSlide.setPower(0);
                slideOtherer.setPower(0);
            } else {

                linearSlideMotor.setPower((liftController.calculate(linearSlideMotor.getCurrentPosition(), target) + f) * multi);
                frontSlide.setPower((liftController.calculate(linearSlideMotor.getCurrentPosition(), target) + f) * multi);
                slideOtherer.setPower((liftController.calculate(slideOtherer.getCurrentPosition(), target) + f) * multi);

            }

            telemetry.addData("slide", linearSlideMotor.getCurrentPosition());
            telemetry.addData("front", frontSlide.getCurrentPosition());
            telemetry.addData("other", slideOtherer.getCurrentPosition());

            armPIDController.setTarget(armTarget);
            armServo.setPower(armPIDController.calculateVelocity());


            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
