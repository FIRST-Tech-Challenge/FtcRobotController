package org.firstinspires.ftc.teamcode.fall2021;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;


@TeleOp(name = "Beeline TeleOp Test", group = "Linear Opmode")
public class Beeline_TeleOp_Test extends LinearOpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;

    public Beeline_TeleOp_Test() {

    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left  = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        double LPower;
        double RPower;


        waitForStart();

        boolean releasedRightBumper = true;
        boolean releasedLeftBumper = true;
        boolean releasedGamePad1 = true;
        boolean releasedA = true;

        boolean toggleGamePad1 = true;

        while (opModeIsActive()) {
            runtime.reset();

            double drive = -gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;

            if(gamepad1.right_bumper) {
                if(releasedRightBumper && releasedLeftBumper) {
                    increaseSpeed(0.05);
                    releasedRightBumper = false;
                }
            } else if(!releasedRightBumper){
                releasedRightBumper = true;
            }

            if(gamepad1.left_bumper){
                if(releasedRightBumper && releasedLeftBumper) {
                    decreaseSpeed(0.05);
                    releasedLeftBumper = false;
                }
            } else if (!releasedLeftBumper){
                releasedLeftBumper = true;
            }

            if(gamepad1.a){
                if(releasedA) {
                    decreaseSpeed(speed / 2.0);
                    releasedA = false;
                }

            } else if(!releasedA){
                increaseSpeed(speed);
                releasedA = true;
            }


            if (gamepad1.x) {
                if (releasedGamePad1){
                    if (toggleGamePad1) {
                        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        toggleGamePad1 = false;
                    } else {
                        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        toggleGamePad1 = true;
                    }
                releasedGamePad1 = false;
                }
            } else if (!releasedGamePad1){
                releasedGamePad1 = true;
            }

            LPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate), -1.0, 1.0) ;
            RPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate), -1.0, 1.0) ;


            left.setPower(LPower);
            right.setPower(RPower);


            telemetry.addData("Left (%.2f)", LPower);
            telemetry.addData("Right (%.2f)", RPower);
            //telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, rotate);
            telemetry.addData("speed:", speed);

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