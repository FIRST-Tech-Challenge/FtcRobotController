package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.StrictMath.abs;


@TeleOp(name="GeneralTeleop", group="Teleop")
//@Disabled
public class GeneralTeleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private Servo atmt = null;
    private DcMotorEx liftL = null;
    private DcMotorEx liftR = null;
    private DcMotorEx vt = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        vt = hardwareMap.get(DcMotorEx.class, "vt");
        atmt = hardwareMap.get(Servo.class, "atmt");
        liftL = hardwareMap.get(DcMotorEx.class, "liftl");
        liftR = hardwareMap.get(DcMotorEx.class, "liftr");

        liftL.setMode(RunMode.RUN_USING_ENCODER);
        liftR.setMode(RunMode.RUN_USING_ENCODER);
        vt.setMode(RunMode.RUN_USING_ENCODER);
        vt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        vt.setDirection(DcMotor.Direction.FORWARD);
        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double lfPower;
            double rfPower;
            double lbPower;
            double rbPower;
            boolean AvtPower;
            double BvtPower;
            double LliftPower;
            double RliftPower;
            double atmtGrip;
            double LiftVelocity = 1120 * 3.75;
            double LiftVelocity2 = 0;

            lfPower = 0.0f ;
            rfPower = 0.0f ;
            lbPower = 0.0f ;
            rbPower = 0.0f ;

            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) > 0.2){
                lfPower = -gamepad1.left_stick_x;
                lbPower = gamepad1.left_stick_x;
            } else
            if (abs(gamepad1.left_stick_y) < 0.2 && abs(gamepad1.left_stick_x) < 0.2){
                lfPower = 0.15 * ((gamepad1.left_stick_y - gamepad1.left_stick_x)/(abs(gamepad1.left_stick_y - gamepad1.left_stick_x))) ;
                lbPower = 0.15 * ((gamepad1.left_stick_y + gamepad1.left_stick_x)/(abs(gamepad1.left_stick_y + gamepad1.left_stick_x)));
            } else
            if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) > 0.2){
                lfPower = gamepad1.left_stick_y - gamepad1.left_stick_x ;
                lbPower = gamepad1.left_stick_y + gamepad1.left_stick_x ;
            } else
            if (abs(gamepad1.left_stick_y) > 0.2 && abs(gamepad1.left_stick_x) < 0.2){
                lfPower = gamepad1.left_stick_y;
                lbPower = gamepad1.left_stick_y;
            }

            if (abs(gamepad1.left_stick_y) < 0.05 && abs(gamepad1.left_stick_x) < 0.05){
                lfPower = 0.0f ;
                lbPower = 0.0f ;
            }

            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) > 0.2){
                rfPower = gamepad1.right_stick_x;
                rbPower = -gamepad1.right_stick_x;
            } else
            if (abs(gamepad1.right_stick_y) < 0.2 && abs(gamepad1.right_stick_x) < 0.2){
                rfPower = 0.15 * ((gamepad1.right_stick_y + gamepad1.right_stick_x)/(abs(gamepad1.right_stick_y + gamepad1.right_stick_x)));
                rbPower = 0.15 * ((gamepad1.right_stick_y - gamepad1.right_stick_x)/(abs(gamepad1.right_stick_y - gamepad1.right_stick_x)));
            } else
            if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) > 0.2){
                rfPower = gamepad1.left_stick_y + gamepad1.right_stick_x ;
                rbPower = gamepad1.left_stick_y - gamepad1.right_stick_x ;
            } else
            if (abs(gamepad1.right_stick_y) > 0.2 && abs(gamepad1.right_stick_x) < 0.2){
                rfPower = gamepad1.right_stick_y;
                rbPower =  gamepad1.right_stick_y;
            }
            if (abs(gamepad1.right_stick_y) < 0.05 && abs(gamepad1.right_stick_x) < 0.05){
                rfPower = 0.0f ;
                rbPower = 0.0f ;
            }

            AvtPower = gamepad2.right_bumper;
            BvtPower = gamepad2.right_trigger;
            LliftPower = gamepad2.left_stick_y ;
            RliftPower = gamepad2.right_stick_y ;

            atmtGrip = Range.clip(gamepad2.left_trigger,0.0, 1.0);

            if (gamepad1.right_bumper){
                lf.setPower(lfPower *0.5);
                rf.setPower(rfPower *0.5);
                lb.setPower(lbPower *0.5);
                rb.setPower(rbPower *0.5);
            } else{
                lf.setPower(lfPower);
                rf.setPower(rfPower);
                lb.setPower(lbPower);
                rb.setPower(rbPower);
            }



            if (LliftPower >= 0.25){
                liftL.setVelocity(LiftVelocity);
            }
            else if (LliftPower <= -0.25){
                liftL.setVelocity(-LiftVelocity);
            }
            else {
                liftL.setVelocity(LiftVelocity2);
            }

            if (RliftPower >= 0.25){
                liftR.setVelocity(LiftVelocity);
            }
            else if (RliftPower <= -0.25){
                liftR.setVelocity(-LiftVelocity);
            }
            else {
                liftR.setVelocity(LiftVelocity2);
            }

            if (AvtPower){
                vt.setVelocity(-770);
            }
            else if (BvtPower >= 0.1){
                vt.setVelocity(770);
            }
            else {
                vt.setVelocity(0);
            }

            atmt.setPosition(atmtGrip);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f),leftback (%.2f), rightback (%.2f)", lfPower, rfPower,lbPower ,rbPower);
            telemetry.update();
        }
    }
}