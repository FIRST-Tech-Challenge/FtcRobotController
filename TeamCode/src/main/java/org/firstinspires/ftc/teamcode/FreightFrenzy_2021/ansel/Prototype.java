// without spin, intake, slide - no ext. hub

package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "Prototype", group = "Linear OpMode")
public class Prototype extends LinearOpMode {

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private Servo Bucket;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.8;
    boolean reverse = false;

    @Override
    public void runOpMode() {

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        Bucket = hardwareMap.get(Servo.class, "Bucket");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean releasedX1 = true;
        boolean releasedLT2 = true;
        boolean releasedA1 = true;
        boolean releasedB1 = true;
        boolean releasedDD1 = true;
        boolean releasedDU1 = true;

        boolean toggleX1 = true;
        boolean toggleLT2 = true;

        while (opModeIsActive()) {
            runtime.reset();

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if(gamepad1.dpad_up) {
                if(releasedDU1) {
                    increaseSpeed(0.05);
                    releasedDU1 = false;
                }
            } else if(!releasedDU1){
                releasedDU1 = true;
            }

            if(gamepad1.dpad_down){
                if(releasedDD1) {
                    decreaseSpeed(0.05);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }

            if(gamepad1.a){
                if(releasedA1) {
                    speed = 0.3;
                    releasedA1 = false;
                }
            } else if(!releasedA1){
                releasedA1 = true;
            }

            if(gamepad1.b){
                if(releasedB1) {
                    speed = 0.8;
                    releasedB1 = false;
                }
            } else if(!releasedB1){
                releasedB1 = true;
            }

            if (gamepad1.x) {
                if (releasedX1) {
                    if (toggleX1) {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        toggleX1 = false;
                    } else {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        toggleX1 = true;
                    }
                    releasedX1 = false;
                }
            } else if (!releasedX1) {
                releasedX1 = true;
            }

            if (gamepad2.left_trigger == 1) {
                if (releasedLT2) {
                    if (Bucket.getPosition() > 0.5) {
                        Bucket.setPosition(0.25);
                    } else {
                        Bucket.setPosition(1);
                    }
                    telemetry.addLine("Bucket");
                    releasedLT2 = false;
                }
            } else if (!releasedLT2) {
                releasedLT2 = true;
            }

            LFPower = Range.clip(gamepad1.left_trigger + speed * (drive + rotate - strafe), -1.0, 1.0);
            LBPower = Range.clip(gamepad1.left_trigger + speed * (drive + rotate + strafe), -1.0, 1.0);
            RFPower = Range.clip(gamepad1.right_trigger + speed * (drive - rotate + strafe), -1.0, 1.0);
            RBPower = Range.clip(gamepad1.right_trigger + speed * (drive - rotate - strafe), -1.0, 1.0);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            telemetry.addData("Servo", "Bucket (%.2f)", Bucket.getPosition(), Bucket.getDirection());
            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);

            telemetry.update();

        }
    }
    private void decreaseSpeed(double s) {
        double decreased = speed - s;
        if (decreased < 0) {
            speed = 0;
            return;
        }
        speed = decreased;
    }

    private void increaseSpeed(double s) {
        double increased = speed + s;
        if (1 < increased) {
            speed = 1;
            return;
        }
        speed = increased;
    }
}