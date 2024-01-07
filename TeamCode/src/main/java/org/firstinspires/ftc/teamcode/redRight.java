package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "redRight", group = "drive")
public class redRight extends LinearOpMode {
    //Declare motors
    Hware robot;

    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;
    public DcMotor arm;
    public DcMotor lift;

    //Declare servos
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo launcher = null;
    public Servo wristLeft = null;
    public Servo wristRight = null;
    ColorSensor leftSensor = null;
    ColorSensor rightSensor = null;

    //Declare Additional variables
    public double ticks = 751.8;
    public double liftNewTarget;
    public double armNewTarget;




    int x, y, z, y2, x2, z2;

    /*
    private void liftControl(double power) {
        Lift1.setPower(power);
        Arm1.setPower(power);
    }


    public void openLeftServo(){
        leftClaw.setPosition(0.73);
    }
    public void openRightServo(){
        rightClaw.setPosition(-0.33); // we will change later
    }
    public void openWristServo(){
        wrist.setPosition(0.93); // we will change later
    }

    public void liftEncodeOpen() {
        int degree = -1;
        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks / degree;
        Lift1.setTargetPosition((int) newTarget);
        Lift1.setPower(-.2);
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void armEncode() {
        int degree = -5;
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armNewTarget = ticks / degree;
        Arm1.setTargetPosition((int) armNewTarget);
        Arm1.setPower(-.2);
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetLift(){
        Lift1.setTargetPosition(0);
        Lift1.setPower(0.2);
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetArm(){
        Arm1.setTargetPosition(0);
        Arm1.setPower(0.2);
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetServo(){
        rightClaw.setPosition(-0.33);
        leftClaw.setPosition(1);
        //launcher.setPosition(0.5);
        wrist.setPosition(0.93);
    }
    public void resetLeftServo(){
        leftClaw.setPosition(0.93);
    }
    public void resetRightServo(){
        rightClaw.setPosition(-0.33); //CHANGE SERVO POSITIONS
    }
    public void resetWristServo(){
        wrist.setPosition(0.445);
    }

     */



    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");

        //Set motor directions
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        //initialize slide motors
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");

        //Set motor modes
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize Servos
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        launcher = hardwareMap.get(Servo.class, "planeLauncher");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        leftSensor = hardwareMap.get(ColorSensor.class,"leftSensor");
        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");

        //Set Zero Power Behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose2 = new Pose2d(12, -36, 270);
        Pose2d startPose1 = new Pose2d(12, -60, 270);

        //drive.setPoseEstimate(startPose);
        TrajectorySequence forward = drive.trajectorySequenceBuilder(startPose1)
                .forward(24)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose2)
                .strafeLeft(11)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //outtake
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //outtake
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {  //driving position
                    robot.leftArm.setTargetPosition(0);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftArm.setPower(0.05);

                    robot.rightArm.setTargetPosition(0);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightArm.setPower(0.05);

                    robot.leftElbow.setPosition(0.625);
                    robot.rightElbow.setPosition(0.375);
                    robot.leftWrist.setPosition(0.52);
                })
                .lineToSplineHeading(new Pose2d(48, -34, Math.toRadians(180)))
                .strafeLeft(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {  //outtake position
                    robot.leftArm.setTargetPosition(-578);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftArm.setPower(0.1);

                    robot.rightArm.setTargetPosition(-578);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightArm.setPower(0.1);

                    robot.leftLift.setTargetPosition(600);
                    robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftLift.setPower(0.3);

                    robot.rightLift.setTargetPosition(600);
                    robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightLift.setPower(0.3);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { // outtake
                    robot.intakeRight.setPower(-0.5);
                    robot.intakeLeft.setPower(-0.5);
                })
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose2)
                .strafeRight(11)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //outtake
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //outtake
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {  //driving position
                    robot.leftArm.setTargetPosition(0);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftArm.setPower(0.05);

                    robot.rightArm.setTargetPosition(0);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightArm.setPower(0.05);

                    robot.leftElbow.setPosition(0.625);
                    robot.rightElbow.setPosition(0.375);
                    robot.leftWrist.setPosition(0.52);
                })
                .lineToSplineHeading(new Pose2d(48, -34, Math.toRadians(180)))
                .strafeRight(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {  //outtake position
                    robot.leftArm.setTargetPosition(-578);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftArm.setPower(0.1);

                    robot.rightArm.setTargetPosition(-578);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightArm.setPower(0.1);

                    robot.leftLift.setTargetPosition(600);
                    robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftLift.setPower(0.3);

                    robot.rightLift.setTargetPosition(600);
                    robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightLift.setPower(0.3);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { // outtake
                    robot.intakeRight.setPower(-0.5);
                    robot.intakeLeft.setPower(-0.5);
                })
                .build();
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose2)
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //outtake
                    robot.intakeRight.setPower(1);
                    robot.intakeLeft.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { //outtake
                    robot.intakeRight.setPower(0);
                    robot.intakeLeft.setPower(0);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {  //driving position
                    robot.leftArm.setTargetPosition(0);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftArm.setPower(0.05);

                    robot.rightArm.setTargetPosition(0);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightArm.setPower(0.05);

                    robot.leftElbow.setPosition(0.625);
                    robot.rightElbow.setPosition(0.375);
                    robot.leftWrist.setPosition(0.52);
                })
                .lineToSplineHeading(new Pose2d(48, -34, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {  //outtake position
                    robot.leftArm.setTargetPosition(-578);
                    robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftArm.setPower(0.1);

                    robot.rightArm.setTargetPosition(-578);
                    robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightArm.setPower(0.1);

                    robot.leftLift.setTargetPosition(600);
                    robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftLift.setPower(0.3);

                    robot.rightLift.setTargetPosition(600);
                    robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightLift.setPower(0.3);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> { // outtake
                    robot.intakeRight.setPower(-0.5);
                    robot.intakeLeft.setPower(-0.5);
                })
                .build();



        waitForStart();


        //telemetry.addData("Type", detectedColor);
        //telemetry.update();

        while (opModeIsActive()) {
            int x = leftSensor.red(); //left
            int y = leftSensor.blue();
            int z = leftSensor.green();
            int x2 = rightSensor.red(); //right
            int y2 = rightSensor.blue();
            int z2 = rightSensor.green();
            telemetry.addData("", x);
            telemetry.addData("", y);
            telemetry.addData("", z);
            telemetry.addData("", x2);
            telemetry.addData("", y2);
            telemetry.addData("", z2);
            telemetry.update();
            drive.followTrajectorySequence(forward);
            if (x > 25 && x > y && x > z) { //left
                drive.followTrajectorySequence(left);
            } else if (x2 > 25 && x2 > y2 && x2 > z2) { //right
                drive.followTrajectorySequence(right);
            } else { // center
                drive.followTrajectorySequence(center);
            }
        }
    };
}


