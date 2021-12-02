package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

import static java.lang.Math.toRadians;

@TeleOp(name = "TELEOP FINAL", group = "Competition")
public class Mecanum_TeleOp_Final extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;

    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.7;
    boolean reverse = false;

    @Override
    public void runOpMode() {


        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide  = hardwareMap.get(DcMotor.class, "Slide");
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

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

        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);
        Push = hardwareMap.get(Servo.class, "Push");


        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        double intakePower = 0;
        double spinPower = 0;
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
        boolean releasedBack =true;
        double rotateSpeed = 0.02;

        boolean toggleX1 = true;
        boolean toggleRB2 = true;
        boolean toggleLB2 = true;
        boolean toggleLT2 = true;

        while (opModeIsActive()) {
            runtime.reset();
            chassis.update();

            // Retrieve your pose
            Pose2d myPose = chassis.getPoseEstimate();
            telemetry.addLine(driveMethod.fieldState(myPose).toString());

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //////////////GAMEPAD 1//////////////
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

            if(gamepad1.a){
                if(releasedA1) {
                    speed = 0.3;
                }
            } else if(!releasedA1){
                releasedA1 = true;
            }

            if(gamepad1.b){
                if(releasedB1) {
                    speed = 0.7;
                }
            } else if(!releasedB1){
                releasedB1 = true;
            }

            if(gamepad2.a){
                if(releasedA2) {
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                }

            } else if(!releasedA2){
                releasedA2 = true;
            }
            if(gamepad2.b){
                if(releasedB2) {
                    Rotate.setPosition(1.0);
                    sleep(500);
                    Slide.setTargetPosition(initialHeight + 750);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                }
            } else if(!releasedB2){
                releasedB2 = true;
            }

            if(gamepad2.y){
                if(releasedY2) {
                    Rotate.setPosition(1.0);
                    sleep(500);
                    Slide.setTargetPosition(initialHeight + 1400);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                }
            } else if(!releasedY2){
                releasedY2 = true;
            }

            if(gamepad2.x){
                if(releasedX2) {
                    Rotate.setPosition(0.03);
                    sleep(300);
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                }
            } else if(!releasedX2){
                releasedX2 = true;
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

            //SHARED HUB
            if(gamepad1.y) {
                if (releasedY1) {
                    if (PoseStorage.state != driveMethod.poseState.BLUE_SHARED_HUB && PoseStorage.state != driveMethod.poseState.RED_SHARED_HUB) {
                        if (PoseStorage.state == driveMethod.poseState.BLUE || PoseStorage.state == driveMethod.poseState.BLUE_WAREHOUSE) {
                            chassis.setPoseEstimate(fieldConstant.SHARED_BLUE_ENTER_POSE);
                            Trajectory sharedTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate(), true)
                                    .lineTo(new Vector2d(64.75, 18))
                                    .build();
                            chassis.followTrajectory(sharedTraj1);
                            Trajectory sharedTraj2 = chassis.trajectoryBuilder(sharedTraj1.end(), true)
                                    .lineToLinearHeading(fieldConstant.SHARED_BLUE_END_POSE)
                                    .build();
                            chassis.followTrajectory(sharedTraj2);
                        } else if (PoseStorage.state == driveMethod.poseState.RED || PoseStorage.state == driveMethod.poseState.RED_WAREHOUSE) {
                            chassis.setPoseEstimate(fieldConstant.SHARED_RED_ENTER_POSE);
                            Trajectory sharedTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate(), true)
                                    .lineTo(new Vector2d(64.75, 18))
                                    .build();
                            chassis.followTrajectory(sharedTraj1);
                            Trajectory sharedTraj2 = chassis.trajectoryBuilder(sharedTraj1.end(), true)
                                    .lineToLinearHeading(fieldConstant.SHARED_RED_END_POSE)
                                    .build();
                            chassis.followTrajectory(sharedTraj2);
                        }
                        speed = 0.3;
                        chassis.updatePoseEstimate();
                        PoseStorage.state = driveMethod.fieldState(chassis.getPoseEstimate());

//                    } else {
//                        if (PoseStorage.state == driveMethod.poseState.BLUE_SHARED_HUB) {
//                            Trajectory leaveTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(64.75, chassis.getPoseEstimate().getY() + 1, toRadians(90)))
//                                    .build();
//                            chassis.followTrajectory(leaveTraj1);
//                            Trajectory leaveTraj2 = chassis.trajectoryBuilder(leaveTraj1.end())
//                                    .forward(24)
//                                    .build();
//                            chassis.followTrajectory(leaveTraj2);
//                        } else if (PoseStorage.state == driveMethod.poseState.RED_SHARED_HUB) {
//                            Trajectory leaveTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(64.75, chassis.getPoseEstimate().getY() - 1, toRadians(-90)))
//                                    .build();
//                            chassis.followTrajectory(leaveTraj1);
//                            Trajectory leaveTraj2 = chassis.trajectoryBuilder(leaveTraj1.end())
//                                    .forward(24)
//                                    .build();
//                            chassis.followTrajectory(leaveTraj2);
//                        }
//                        speed = 0.7;
//                        chassis.updatePoseEstimate();
//                        PoseStorage.state = driveMethod.fieldState(chassis.getPoseEstimate());
//                    }
                } else if (!releasedY1) {
                    releasedY1 = true;
                }
            }

            //////////////GAMEPAD 2//////////////
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
                if(releasedBack){
                    if(rotateSpeed==0.02)
                        rotateSpeed =0.05;
                    else if (rotateSpeed==0.05)
                        rotateSpeed =0.02;
                }else if (!releasedBack){
                    releasedBack = true;
                }
            }
            if (gamepad2.dpad_left) {
                if (releasedDL2){
//                    if(spinPower != 0) {
//                        spinPower -= 0.05;
//                    }
                    while(Rotate.getPosition() <= 1.0 && gamepad2.dpad_left) {
                        Rotate.setPosition(Rotate.getPosition()+rotateSpeed);
                        sleep(40);
                    }
                    releasedDL2 = false;
                }
            } else if (!releasedDL2){
                releasedDL2 = true;
            }
            if (gamepad2.dpad_right) {
                if (releasedDR2){
//                    if(spinPower != 0) {
//                        spinPower += 0.05;
//                    }
                    while(Rotate.getPosition() >= 0.03 && gamepad2.dpad_right) {
                        Rotate.setPosition(Rotate.getPosition()-rotateSpeed);
                        sleep(40);
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
                        telemetry.addLine("SPIN STARTS");
                        toggleRB2 = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
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
                        telemetry.addLine("SPIN STARTS REVERSE");
                        toggleLB2 = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
                        toggleLB2 = true;
                    }
                    releasedLB2 = false;
                }
            } else if (!releasedLB2){
                releasedLB2 = true;
            }

            if(gamepad2.right_trigger == 1 && Rotate.getPosition() > 0.4){
                if (releasedRT2){
                    Push.setPosition(0);
                    releasedRT2 = false;
                }
            } else if (!releasedRT2){
                Push.setPosition(0.4);
                releasedRT2 = true;
            }

            if(gamepad2.left_trigger == 1){
                if (releasedLT2){
                    if (toggleLT2 && !Slide.isBusy()) {
                        Rotate.setPosition(0.03);
                        telemetry.addLine("ROTATE STARTS");
                        toggleLT2 = false;
                    } else {
                        Rotate.setPosition(1.0);
                        telemetry.addLine("ROTATE STOPS");
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

            telemetry.addData("Servo","Rotate (%.2f), Push (%.2f)", Rotate.getPosition(), Push.getPosition());
            telemetry.addLine("Intake: " + intakePower);
            telemetry.addLine("Spin: " + spinPower);
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);

            telemetry.update();

        }
    }

    void twoPhaseSpin(boolean isReversed,double startingSpeed) {
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

}