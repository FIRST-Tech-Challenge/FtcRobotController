package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Team 23974 A.S.T.R.O. Vikings, water 2024-2025
@Autonomous(name="Second Auto", group ="AHHHHHHHH", preselectTeleOp = "Teleop To Use :))))")
public class ImperfectAuto extends LinearOpMode {
    final double ARMROTMULT = 60.0/43.0;

    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
    Servo wrist = null;
    Servo grabber = null;


    double actuatorPos = 0;
    double armRotPos = 0;
    double wristRotPos=0.65;

    SampleMecanumDrive drive;
    double grabOpen = 0.75;
    double grabClose = 0.4;


//    void controlLinearActuator() {
//        linearActuator.setTargetPosition((int) actuatorPos);
//        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//
//    void controlGrabber(boolean open) {
//        if (open){
//            grabber.setPosition(grabOpen);
//        } else {
//            grabber.setPosition(grabClose);
//        }
//    }
//
//    double armPos = 0;
//
//    void controlBothArmExtenders() {
//        armLifterLeft.setTargetPosition((int) (armPos*ARMROTMULT));
//        armLifterRight.setTargetPosition(-(int) (armPos*ARMROTMULT));
//        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    void controlArmRotate() {
//        armRotate.setTargetPosition((int) armRotPos);
//    }
//
//    void controlWristRotate(){
//        wrist.setPosition(wristRotPos);
//    }

    @Override
    public void runOpMode() {
        telemetry.addData("start of run opmode","start of run opmode");
        telemetry.update();
        armLifterLeft = hardwareMap.dcMotor.get("armLifterLeft");
        armLifterRight = hardwareMap.dcMotor.get("armLifterRight");
        armRotate = hardwareMap.dcMotor.get("armRotate");
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.servo.get("grabber");

        grabber.setDirection(Servo.Direction.FORWARD);

        armLifterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double slowerVelocity = 25; // 25 is half ish of the full speed
        TrajectoryVelocityConstraint velCons = SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accCons = SampleMecanumDrive.getAccelerationConstraint(slowerVelocity);

//        armLifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armLifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("before loop","before loop");
        while (opModeIsActive()) {

            // Push telemetry to the Driver Station.
            telemetry.addData("in loop","in loop");
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }
        telemetry.addData("after loop","after loop");
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d();
//
//        Trajectory tr1 = drive.trajectoryBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(13.13,2.940,Math.toRadians(358.58)), 0)
//                .build();
//        Trajectory tr2 = drive.trajectoryBuilder(tr1.end())
//                .splineToLinearHeading(new Pose2d(12.467,2.905,Math.toRadians(358.611)),0)
//                .build();
//        Trajectory tr3 = drive.trajectoryBuilder(tr2.end())
//                .splineToLinearHeading(new Pose2d(21.138,38.39,Math.toRadians(354.715)),0)
//                .build();
//        Trajectory tr4 = drive.trajectoryBuilder(tr3.end())
//                .splineToLinearHeading(new Pose2d(15.863,39.26,Math.toRadians(2.958)),0)
//                .build();
//        Trajectory tr5 = drive.trajectoryBuilder(tr4.end())
//                .splineToLinearHeading(new Pose2d(4.962,44.514,Math.toRadians(319.379)),0)
//                .build();
//        Trajectory tr6 = drive.trajectoryBuilder(tr5.end())
//                .splineToSplineHeading(new Pose2d(32.956,14.2,Math.toRadians(232.615)),0)
//                .build();
//        Trajectory tr7 = drive.trajectoryBuilder(tr6.end())
//                .splineToLinearHeading(new Pose2d(-6.3,-44.726,Math.toRadians(237.792)),0)
//                .build();
//        Trajectory tr8 = drive.trajectoryBuilder(tr7.end())
//                .splineToLinearHeading(new Pose2d(10.072,-56.662,Math.toRadians(235.429)),0)
//                .build();
//        Trajectory tr9 = drive.trajectoryBuilder((tr8.end()))
//                .splineToLinearHeading(new Pose2d(17.195,-47.265,Math.toRadians(233.733)),0)
//                .build();



        waitForStart(); /*****  DON'T RUN ANY MOTOR MOVEMENT ABOVE THIS LINE!! You WILL get PENALTIES! And it's UNSAFE! *****/
        if (isStopRequested()) return;
//
//        armLifterLeft.setTargetPosition((int) armPos);
//        armLifterRight.setTargetPosition((int) armPos);
//        linearActuator.setTargetPosition((int) actuatorPos);
//        armRotate.setTargetPosition((int) armRotPos);
//        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        armLifterRight.setPower(1);
//        armLifterLeft.setPower(1);
//        linearActuator.setPower(1);
//        armRotate.setPower(1);
        telemetry.addData("stuff starts","stuff starts");
//        /***** start of manual code running or initiation or whatever *****/
//        controlArmRotate();
//        controlWristRotate();
//        //move arm to set on chambers
//        armRotPos = -2314.8567;
//        wristRotPos = 0.865;
//        controlWristRotate();
//        controlArmRotate();
//        sleep(1000);
//        //move to chambers
//        drive.followTrajectory(tr1);
//        telemetry.addData("tr1","tr1");
//        //set on  chamber
//        wristRotPos = 0.81;
//        armRotPos = -1682.074;
//        controlWristRotate();
//        controlArmRotate();
//        drive.followTrajectory(tr2);
//        telemetry.addData("tr2","tr2");
//        //back up and move arm to safety
//        wristRotPos = 0.625;
//        armRotPos = 0;
//        controlArmRotate();
//        controlWristRotate();
//        drive.followTrajectory(tr3);
//        sleep(1000);
//        //go to pick up sample
//        drive.followTrajectory(tr4);
//        //raise arm
//        armRotPos = -2127.73;
//        controlArmRotate();
//        sleep(1000);
//        //move wrist down
//        wristRotPos = 0.875;
//        controlWristRotate();
//        sleep(1000);
//        controlGrabber(true);
//        //move arm down to grab it
//        armRotPos = -1090;
//        controlArmRotate();
//        sleep(400);
//        controlGrabber(false);
//        sleep(1000);
//        //lift arm to place in high basket
//        wristRotPos = 0.74;
//        armRotPos = -3043.53;
//        armPos = -2032.94;
//        controlArmRotate();
//        controlWristRotate();
//        controlBothArmExtenders();
//        sleep(2000);
//        //drive to high basket
//        drive.followTrajectory(tr5);
//        //drop
//        controlGrabber(true);
//        sleep(500);
//        controlGrabber(false);
//        //drive out of way
//        drive.followTrajectory(tr6);
//        armRotPos = -209.302;
//        wristRotPos = 0.625;
//        armPos = -5;
//        controlArmRotate();
//        controlWristRotate();
//        controlBothArmExtenders();
    //        drive.followTrajectory(tr7);
    //        drive.followTrajectory(tr8);
    //        drive.followTrajectory(tr9);
    //        armPos = -2053.53;
    //        controlBothArmExtenders();



sleep(2000);
        /***** end of manual code running or initiation or whatever *****/
    }



    public Vector2d poseToVector(Pose2d Pose) {
        return new Vector2d(Pose.getX(), Pose.getY());
    }
    public Pose2d vectorToPose(Vector2d Vector, Double Heading) {
        return new Pose2d(Vector.getX(), Vector.getY(), Math.toRadians(Heading));
    }

    public Pose2d vectorToPose(Vector2d Vector, int Heading) {
        return new Pose2d(Vector.getX(), Vector.getY(), Math.toRadians(Heading));
    }

}