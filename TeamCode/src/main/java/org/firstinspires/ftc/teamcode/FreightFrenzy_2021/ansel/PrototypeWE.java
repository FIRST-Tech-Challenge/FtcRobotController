/*
this is untested
*/
package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.ansel;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "PrototypeWE", group = "Linear OpMode")
public class PrototypeWE extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Bucket = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.7;
    boolean reverse = false;

    @Override
    public void runOpMode() {
        //get hardware

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide  = hardwareMap.get(DcMotor.class, "Slide");
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");
        Bucket = hardwareMap.get(Servo.class, "Bucket");

        //set direction of hardware

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Bucket.setDirection(Servo.Direction.FORWARD);

        //set power var

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        double intakePower = 0;
        double spinPower = 0;
        double slidePower = 0.8;
        double poseSpeed = 0.3;
        double initialSpeed = 0.7;

        int initialHeight = Slide.getCurrentPosition();

        //Read Position From Auto
        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);
        chassis.setPoseEstimate(PoseStorage.currentPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean releasedRB1 = true;
        boolean releasedLB1 = true;
        boolean releasedX1 = true;
        boolean releasedRB2 = true;
        boolean releasedLB2 = true;
        boolean releasedLT2 = true;
        boolean releasedRT2 = true;
        boolean releasedA1 = true;
        boolean releasedA2 = true;
        boolean releasedB1 = true;
        boolean releasedB2 = true;
        boolean releasedY1 = true;
        boolean releasedY2 = true;
        boolean releasedX2 = true;
        boolean releasedDD1 = true;
        boolean releasedDD2 = true;
        boolean releasedDL2 = true;
        boolean releasedDU1 = true;
        boolean releasedDU2 = true;
        boolean releasedDR2 = true;
        boolean releasedBack2 = true;
        boolean releasedStart2 = true;
        boolean releasedStart1 = true;
        boolean releasedBack1 = true;

        //change if needed
        double bucketSpeed = 1.0;

        boolean toggleX1 = true;
        boolean toggleRB2 = true;
        boolean toggleLB2 = true;
        boolean toggleLT2 = true;

        while (opModeIsActive()) {
            runtime.reset();
            chassis.update();

            // Retrieve current pose state
            Pose2d myPose = chassis.getPoseEstimate();
            telemetry.addLine(driveMethod.fieldState(myPose).toString());

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Gamepad 1

            if(gamepad1.dpad_up) {
                if(releasedDU1) {
                    //speed = increasePower(speed, 0.05);
                    increaseSpeed(0.05);
                    releasedDU1 = false;
                }
            } else if(!releasedDU1){
                releasedDU1 = true;
            }

            if(gamepad1.dpad_down){
                if(releasedDD1) {
                    //speed = decreasePower(speed, 0.05);
                    decreaseSpeed(0.05);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }
            if (gamepad1.left_bumper) {
                if (releasedLB1 && Slide.getCurrentPosition() < initialHeight + 30){
                    intakePower = 1.0;
                    telemetry.addLine("INTAKE STARTS");
                    releasedLB1 = false;
                }
            } else if (!releasedLB1){
                intakePower = 0;
                telemetry.addLine("INTAKE STOPS");
                releasedLB1 = true;
            }
            if (gamepad1.right_bumper) {
                if (releasedRB1 && Slide.getCurrentPosition() < initialHeight + 30){
                    intakePower = -1.0;
                    telemetry.addLine("INTAKE REVERSE STARTS");
                    releasedRB1 = false;
                }
            } else if (!releasedRB1){
                telemetry.addLine("INTAKE STOPS");
                intakePower = 0;
                releasedRB1 = true;
            }

            if(gamepad1.back){
                if(releasedBack1){
                    if (poseSpeed == 0.3) {
                        poseSpeed = 0.7;
                        releasedBack1 = false;
                    }
                    else if (poseSpeed == 0.7) {
                        poseSpeed = 0.3;
                        releasedBack1 = false;
                    }
                } else if (!releasedBack1){
                    releasedBack1 = true;
                }
            }

            if(gamepad1.start){
                if(releasedStart1) {
                    if (speed < 0.5) {
                        speed = 0.7;
                    } else if (speed > 0.5){
                        speed = 0.3;
                    }
                    releasedStart1 = false;
                }
            } else if(!releasedStart1){
                releasedStart1 = true;
            }

            if(gamepad1.b){
                if(releasedB1) {
                    initialSpeed = speed;
                    drawTrajectory.gotoPlate();
                    speed = poseSpeed;
                    drawTrajectory.updatePose();
                }
            } else if(!releasedB1){
                speed = initialSpeed;
                releasedB1 = true;
            }

            if (gamepad1.x) {
                if (releasedX1){
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
            } else if (!releasedX1){
                releasedX1 = true;
            }

            //goto shared hub
            if(gamepad1.y) {
                if (releasedY1) {
                    initialSpeed = speed;
                    drawTrajectory.gotoSharedHub();
                    speed = poseSpeed;
                    drawTrajectory.updatePose();
                    releasedY1 = false;
                } else if (!releasedY1) {
                    speed = initialSpeed;
                    releasedY1 = true;
                }
            }

            //Gamepad 2

            if(gamepad2.a){
                if(releasedA2) {
                    bucketWithSpeed(0, bucketSpeed);
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(slidePower);
                    releasedA2 = false;
                }
            } else if(!releasedA2){
                releasedA2 = true;
            }
            if(gamepad2.b){
                if(releasedB2) {
                    Slide.setTargetPosition(initialHeight + 750);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(slidePower);
                    releasedB2 = false;
                }
            } else if(!releasedB2){
                releasedB2 = true;
            }

            if(gamepad2.y){
                if(releasedY2) {
                    Slide.setTargetPosition(initialHeight + 1400);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(slidePower);
                    releasedY2 = false;
                }
            } else if(!releasedY2){
                releasedY2 = true;
            }

            if(gamepad2.x){
                if(releasedX2) {
                    bucketWithSpeed(0, bucketSpeed);
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(slidePower);
                    releasedX2 = false;
                }
            } else if(!releasedX2){
                releasedX2 = true;
            }

            if (gamepad2.dpad_up) {
                if (releasedDU2){
                    Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Slide.setPower(0.5);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower(0);
                releasedDU2 = true;
            }
            if (gamepad2.dpad_down) {
                if (releasedDD2){
                    Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Slide.setPower(-0.5);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2){
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower(0);
                releasedDD2 = true;
            }

            if(gamepad2.back){
                if(releasedBack2){
                    if (bucketSpeed == 0.50) {
                        bucketSpeed = 1.00;
                        releasedBack2 = false;
                    }
                    else if (bucketSpeed == 1.00) {
                        bucketSpeed = 0.50;
                        releasedBack2 = false;
                    }
                } else if (!releasedBack2){
                    releasedBack2 = true;
                }
            }

            if(gamepad2.start){
                if(releasedStart2){
                    if (slidePower == 0.8) {
                        slidePower = 1.0;
                        releasedStart2 = false;
                    }
                    else if (slidePower == 1.0) {
                        slidePower = 0.8;
                        releasedStart2 = false;
                    }
                } else if (!releasedStart2){
                    releasedStart2 = true;
                }
            }

            if (gamepad2.dpad_left) {
                if (releasedDL2){
                    if(spinPower != 0) {
                        spinPower -= 0.05;
                    }
                    releasedDL2 = false;
                }
            } else if (!releasedDL2){
                releasedDL2 = true;
            }
            if (gamepad2.dpad_right) {
                if (releasedDR2){
                   if(spinPower != 0) {
                        spinPower += 0.05;
                    }
                    releasedDR2 = false;
                }
            } else if (!releasedDR2){
                releasedDR2 = true;
            }

            if (gamepad2.right_bumper) {
                if (releasedRB2){
                    if (toggleRB2) {
                        spinPower = 0.70;
                        //twoPhaseSpin(false, 0.7);
                        telemetry.addLine("Spin Starts - Blue");
                        toggleRB2 = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("Spin Stops");
                        toggleRB2 = true;
                    }
                    releasedRB2 = false;
                }
            } else if (!releasedRB2){
                releasedRB2 = true;
            }

            if (gamepad2.left_bumper) {
                if (releasedLB2){
                    if (toggleLB2) {
                        spinPower = -0.70;
                        //twoPhaseSpin(true, 0.7);
                        telemetry.addLine("Spin Starts - Red");
                        toggleLB2 = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("Spin Stops");
                        toggleLB2 = true;
                    }
                    releasedLB2 = false;
                }
            } else if (!releasedLB2){
                releasedLB2 = true;
            }

            /* if(gamepad2.right_trigger == 1 && Rotate.getPosition() > 0.4){
                if (releasedRT2){
                    Push.setPosition(0);
                    releasedRT2 = false;
                }
            } else if (!releasedRT2){
                Push.setPosition(0.4);
                releasedRT2 = true;
            } */

            if(gamepad2.left_trigger == 1){
                if (releasedLT2){
                    if (toggleLT2 && !Slide.isBusy()) {
                        Bucket.setPosition(0.03);
                        telemetry.addLine("Bucket - Initial Position");
                        toggleLT2 = false;
                    } else {
                        Bucket.setPosition(1.0);
                        telemetry.addLine("Bucket - Final Position");
                        toggleLT2 = true;
                    }
                    releasedLT2 = false;
                }
            } else if (!releasedLT2){
                releasedLT2 = true;
            }

            LFPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate - strafe), -1.0, 1.0) ;

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);
            Intake.setPower(intakePower);
            Spin.setPower(spinPower);

            telemetry.addData("Servo","Bucket Position (%.2f), Direction (%.2f), Speed (%.2f)", Bucket.getPosition(), Bucket.getDirection(), bucketSpeed);
            telemetry.addLine("Intake: " + intakePower);
            telemetry.addData("Slide", "Current (%.2f), Target (%.2f), Power (%.2f)", Slide.getCurrentPosition(), Slide.getTargetPosition(), slidePower);
            telemetry.addData("Spin", "Power", spinPower);
            telemetry.addData("Pose", "Speed", poseSpeed);
            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);

            telemetry.update();

        }
    }

    void twoPhaseSpin(boolean isReversed, double startingSpeed) {
        double reverseFactor = 1;
        if(isReversed){
            reverseFactor = -1;
        }
        ElapsedTime tSpin = new ElapsedTime();
        double spinPower = startingSpeed * reverseFactor;
        while (tSpin.milliseconds() < 800){
            Spin.setPower(spinPower);
        }
        while (tSpin.milliseconds() < 1500){
            spinPower = Range.clip(spinPower * tSpin.milliseconds()/800.0, -1.0, 1.0);
            Spin.setPower(spinPower);
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

    private double decreasePower(double p, double r) {
        double decreased = p - r;
        if (decreased < 0) {
            p = 0;
            return p;
        }
        p = decreased;
        return p;
    }

    private double increasePower(double p, double r) {
        double increased = p + r;
        if (1 < increased) {
            p = 1;
            return p;
        }
        p = increased;
        return p;
    }

    private void bucketWithSpeed(double targetPos, double factor){
        //1 -> 10 0.5 -> 5
        if(factor == 1.0){
            Bucket.setPosition(targetPos);
        }
        double currentPos = Bucket.getPosition();
        double interval = 0.05 * factor;
        while (targetPos > Bucket.getPosition()){
            Bucket.setPosition(Bucket.getPosition() + interval);
            sleep(30);
        }
        while (targetPos < Bucket.getPosition()){
            Bucket.setPosition(Bucket.getPosition() - interval);
            sleep(30);
        }
    }

}