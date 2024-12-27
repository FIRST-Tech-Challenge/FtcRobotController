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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Team 23974 A.S.T.R.O. Vikings, water 2024-2025
@Autonomous(name="First Auto", group ="AHHHHHHHH", preselectTeleOp = "Teleop To Use :))))")
public class PerfectAuto extends LinearOpMode {

    //other motors
    DcMotor armLifterLeft = null;
    DcMotor armLifterRight = null;
    DcMotor armRotate = null;
    DcMotor linearActuator = null;
    //servos
//    CRServo spool = null;
    Servo wrist = null;
    CRServo sampPickUpLeft = null;
    CRServo sampPickUpRight = null;


    double actuatorPos = 0;
    double armRotPos = 0;
    double wristRotPos=0;

    SampleMecanumDrive drive;


    void controlLinearActuator() {
        linearActuator.setTargetPosition((int) actuatorPos);
        linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void controlBothGrabbers(int i) {
        sampPickUpLeft.setPower(i);
        sampPickUpRight.setPower(i);
    }

    double armPos = 0;

    void controlBothArmExtenders() {
        armLifterLeft.setTargetPosition((int) armPos);
        armLifterRight.setTargetPosition(-(int) armPos);
        armLifterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLifterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void controlArmRotate() {
        armRotate.setTargetPosition((int) armRotPos);
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
        wrist = hardwareMap.servo.get("wrist");
        sampPickUpLeft = hardwareMap.crservo.get("sampPickUpLeft");
        sampPickUpRight = hardwareMap.crservo.get("sampPickUpRight");

        sampPickUpLeft.setDirection(CRServo.Direction.FORWARD);
        sampPickUpRight.setDirection(CRServo.Direction.REVERSE);

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
        armRotate.setTargetPosition((int) armRotPos);
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
        Pose2d startPose = new Pose2d();

        Trajectory tr1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(22.572,0), 0)
                .build();
        Trajectory tr2 = drive.trajectoryBuilder(tr1.end())
                .back(10)
                .build();
        Trajectory tr3 = drive.trajectoryBuilder(tr2.end())
                .splineToConstantHeading(new Vector2d(19.32,38.39),0)
                .build();



        waitForStart(); /*****  DON'T RUN ANY MOTOR MOVEMENT ABOVE THIS LINE!! You WILL get PENALTIES! And it's UNSAFE! *****/
        if (isStopRequested()) return;

        /***** start of manual code running or initiation or whatever *****/
        drive.followTrajectory(tr1);
        armRotPos = -1053.25;
        wristRotPos = 0.71;
        controlWristRotate();
        controlArmRotate();
        sleep(1000);
        wristRotPos = 0.65;
        armRotPos = -659.915;
        controlWristRotate();
        controlArmRotate();
        drive.followTrajectory(tr2);
        wristRotPos = 1;
        armRotPos = 0;
        controlArmRotate();
        controlWristRotate();
        sleep(1000);
        drive.followTrajectory(tr3);
        armRotPos = -600;
        controlArmRotate();
        sleep(1000);
        wristRotPos = 0.2;
        controlWristRotate();
        sleep(1000);
        controlBothGrabbers(1);
        armRotPos = -485;
        controlArmRotate();
        sleep(200);
        controlBothGrabbers(0);
        sleep(1000);



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