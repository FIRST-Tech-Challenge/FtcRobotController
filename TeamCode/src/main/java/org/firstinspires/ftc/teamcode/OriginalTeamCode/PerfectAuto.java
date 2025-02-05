package org.firstinspires.ftc.teamcode.OriginalTeamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// Team 23974 A.S.T.R.O. Vikings, water 2024-2025
@Autonomous(name="Specimen And High Baskets Auto", group ="AHHHHHHHH", preselectTeleOp = "Teleop To Use :))))")
public class PerfectAuto extends LinearOpMode {
    final double OPEN = 0.75;
    final double CLOSE = 0.4;
    final double ARMROTMULT = 1;

    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
//    CRServo spool = null;
    ServoImplEx wrist = null;
    Servo grabber = null;


    double actuatorPos = 0;
    double armRotPos = 0;
    double wristRotPos=0;

    SampleMecanumDrive drive;


    void controlLinearActuator() {
        linearActuator.setTargetPosition((int) actuatorPos);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void controlGrabber(double i) {
        grabber.setPosition(i);
    }

    double armPos = 0;

    void controlBothArmExtenders() {
        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition(-(int) armPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void controlArmRotate() {
        armRotate.setTargetPosition((int) (armRotPos * ARMROTMULT));
    }

    void controlWristRotate(){
        wrist.setPosition(wristRotPos);
    }

    @Override
    public void runOpMode() {
        armLifterLeft = hardwareMap.dcMotor.get("armLifterLeft");
        armLifterRight = hardwareMap.dcMotor.get("armLifterRight");
        armRotate = hardwareMap.dcMotor.get("armRotate");
        linearActuator = hardwareMap.dcMotor.get("linearActuator");
        wrist = hardwareMap.get(ServoImplEx.class,"wrist");
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        grabber = hardwareMap.servo.get("grabber");


        armLifterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double slowerVelocity = 25; // 25 is half ish of the full speed
        TrajectoryVelocityConstraint velCons = SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accCons = SampleMecanumDrive.getAccelerationConstraint(slowerVelocity);

        armLifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLifterRight.setPower(1);
        armLifterLeft.setPower(1);
        linearActuator.setPower(1);
        armRotate.setPower(1);

        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition((int) armPos);
        linearActuator.setTargetPosition((int) actuatorPos);
        armRotate.setTargetPosition((int) (armRotPos*ARMROTMULT));
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {

            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);

        }

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,0);

        Trajectory tr1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20.889,0), 0)
                .build();
        Trajectory tr2 = drive.trajectoryBuilder(tr1.end())
                .back(7)
                .build();
        Trajectory tr3 = drive.trajectoryBuilder(tr2.end())
                .splineTo(new Vector2d(20,38.5), 0)
                .build();
        Trajectory tr4 = drive.trajectoryBuilder(tr3.end())
                .splineToLinearHeading(new Pose2d(8,48,Math.toRadians(-225)),0)
                .build();
        Trajectory tr5 = drive.trajectoryBuilder(tr4.end())
                .splineToLinearHeading(new Pose2d(12,41,0),0)
                .build();
        Trajectory tr6 = drive.trajectoryBuilder(tr5.end())
                .splineToConstantHeading(new Vector2d(20,50),0)
                .addTemporalMarker(1.25, ()->{
                    armRotPos = -611;
                    wristRotPos = 0.57;
                    armPos = 30;
                    controlBothArmExtenders();
                    controlArmRotate();
                    controlWristRotate();
                })
                .build();
        Trajectory tr7 = drive.trajectoryBuilder(tr6.end())
                .splineToSplineHeading(new Pose2d(5,50,Math.toRadians(130)),0)
                .build();
        Trajectory tr8 = drive.trajectoryBuilder(tr7.end())
                .back(8)
                .splineToSplineHeading(new Pose2d(54,30,Math.toRadians(90)),0)
                .addTemporalMarker(0.75,()->{
                    controlArmRotate();
                    controlWristRotate();
                    controlBothArmExtenders();
                })
                .build();
        Trajectory tr9 = drive.trajectoryBuilder(tr8.end())
                .splineToConstantHeading(new Vector2d(54,6),0)
                .addTemporalMarker(.25,()->{
                    armPos = -3400;
                    controlBothArmExtenders();
                })
                .build();

        waitForStart(); /*****  DON'T RUN ANY MOTOR MOVEMENT ABOVE THIS LINE!! You WILL get PENALTIES! And it's UNSAFE! *****/
        if (isStopRequested()) return;

        /***** start of manual code running or initiation or whatever *****/
        controlGrabber(CLOSE);
        armRotPos = -2462.416;
        wristRotPos = 0.615;
        controlWristRotate();
        controlArmRotate();
        sleep(600);
        drive.followTrajectory(tr1);
        wristRotPos = 0.63;
        armRotPos = -2392.33;
        controlArmRotate();
        controlWristRotate();
        sleep(100);
        drive.followTrajectory(tr2);
        controlGrabber(OPEN);
//        sleep(500);
        drive.followTrajectory(tr3);
        armRotPos = -611;
        wristRotPos = 0.57;
        controlArmRotate();
        controlWristRotate();
        sleep(600);
        controlGrabber(CLOSE); //first sample
        sleep(1000);
        armRotPos = -2930;
        wristRotPos = 0.63;
        armPos = -4015.5;
        controlWristRotate();
        controlArmRotate();
        controlBothArmExtenders();
        sleep(300);
        drive.followTrajectory(tr4);
        controlGrabber(OPEN);
        drive.followTrajectory(tr5);
        sleep(300);
        drive.followTrajectory(tr6);
        sleep(600);
        controlGrabber(CLOSE); //2nd sample
        sleep(900);
        armRotPos = -2964.3;
        wristRotPos = 0.63;
        armPos = -4032.1;
        controlWristRotate();
        controlArmRotate();
        controlBothArmExtenders();
//        sleep(250);
        drive.followTrajectory(tr7);
        controlGrabber(OPEN);
        sleep(300);
        armPos = 0;
//        controlBothArmExtenders();
//        armPos = -3400;
        wristRotPos = 0.4;
        armRotPos = 20;
        drive.followTrajectory(tr8);
        drive.followTrajectory(tr9);


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