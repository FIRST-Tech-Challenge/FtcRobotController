package org.firstinspires.ftc.teamcode.pinpoint.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Pinpoint Navigation Sample", group="Pinpoint")
//@Disabled

public class SensorPinpointDriveToPoint extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DRIVE_TO_TARGET_10,
        DRIVE_TO_TARGET_11,
        DRIVE_TO_TARGET_12,
        DRIVE_TO_TARGET_13,
        DRIVE_TO_TARGET_14,
        DRIVE_TO_TARGET_15,
        DRIVE_TO_TARGET_16,
        DRIVE_TO_TARGET_17


    }
//    static final Pose2D TARGET_FORWARD = new Pose2D(DistanceUnit.MM,2000,0,AngleUnit.DEGREES,0);
//    static final Pose2D TARGET_LEFT = new Pose2D(DistanceUnit.MM,0,200,AngleUnit.DEGREES,0);
//    static final Pose2D TARGET_ROTATE = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,-90);
//    static final Pose2D TARGET_ANTI_ROTATE = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,-270);

    //   static final Pose2D TARGET_FORWARD = new Pose2D(DistanceUnit.MM,1000,1000,AngleUnit.DEGREES,0);
    //   static final Pose2D TARGET_LEFT = new Pose2D(DistanceUnit.MM,0,1000,AngleUnit.DEGREES,0);
    //   static final Pose2D TARGET_ROTATE = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,-90);
    //   static final Pose2D TARGET_ANTI_ROTATE = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,-270);

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH,48,64,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, 35, 64, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH, 35, 24, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, 54.5, 24, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, 54.5, 64, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_6 = new Pose2D(DistanceUnit.INCH, 59, 64, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_7 = new Pose2D(DistanceUnit.INCH, 39, 64, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_8 = new Pose2D(DistanceUnit.INCH, 39, 24, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_9 = new Pose2D(DistanceUnit.INCH, 65.5, 24, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_10 = new Pose2D(DistanceUnit.INCH, 65.5, 64, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_11 = new Pose2D(DistanceUnit.INCH, 59, 64, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_12 = new Pose2D(DistanceUnit.INCH, 59, 24, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_13 = new Pose2D(DistanceUnit.INCH, 71, 24, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_14 = new Pose2D(DistanceUnit.INCH, 71, 52, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_15 = new Pose2D(DistanceUnit.INCH, 32, 52, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_16 = new Pose2D(DistanceUnit.INCH, 32, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_17 = new Pose2D(DistanceUnit.INCH, 17, 0, AngleUnit.DEGREES, -90.00);


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(22.0, -111.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.addData("Current Position X", odo.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("Current Position Y", odo.getPosition().getY(DistanceUnit.MM));
        telemetry.addData("Current Heading", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 0.0, 64.0, AngleUnit.DEGREES, 0);

        odo.setPosition(initialPose);
        odo.update();

        while (opModeIsActive()) {

            telemetry.addData("stateMachine", stateMachine);
            telemetry.addData("Current Position In Loop X", odo.getPosition().getX(DistanceUnit.MM));
            telemetry.addData("Current Position In Loop Y", odo.getPosition().getY(DistanceUnit.MM));
            telemetry.addData("Current Heading In Loop", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();

            telemetry.addData("Current Position In Loop X : " + odo.getPosition().getX(DistanceUnit.MM),
                    " Current Position In Loop Y : " + odo.getPosition().getY(DistanceUnit.MM),
                    " Current Position In Loop Heading : " + odo.getPosition().getHeading(AngleUnit.DEGREES),
                    " stateMachine : " + stateMachine);

            odo.update();

            // Test forward motion
//            nav.driveTo(odo.getPosition(), TARGET_FORWARD, 0.3, 3);
//            telemetry.addLine("There!");
//            stateMachine = StateMachine.AT_TARGET;

//            // Test left motion
//            nav.driveTo(odo.getPosition(), TARGET_LEFT, 0.3, 3);
//            telemetry.addLine("There!");
//            stateMachine = StateMachine.AT_TARGET;
//
//            Pose2D rotatingPose = null;
//
//            double xPosition = 0.0;
//            double yPosition = 0.0;
//            double heading = 0.0;
//
////            // Test clockwise motion
//            int count = 0;
//            while(count < 4) {
//
//                telemetry.addData("Count", count);
//                telemetry.addData("Heading value", heading);
//
//                rotatingPose = new Pose2D(DistanceUnit.MM, xPosition, yPosition, AngleUnit.DEGREES, heading + -90.0);
//
//                telemetry.addData("rotatingPose", rotatingPose.getHeading(AngleUnit.DEGREES));
//                telemetry.update();
//
//                nav.driveTo(odo.getPosition(), rotatingPose, 0.3, 3);
////                telemetry.addLine("There!");
//                count++;
////                stateMachine = StateMachine.AT_TARGET;
//            }

            // Test clockwise rotation
//            nav.driveTo(odo.getPosition(), TARGET_ROTATE, 0.3, 3);
//            telemetry.addLine("There!");
//            stateMachine = StateMachine.AT_TARGET;
//
//            // Test anti clockwise rotation
//            nav.driveTo(odo.getPosition(), TARGET_ANTI_ROTATE, 0.3, 3);
//            telemetry.addLine("There!");
//            stateMachine = StateMachine.AT_TARGET;


            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.3, 1)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.3, 1)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), TARGET_3, 0.3, 1)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(),TARGET_4,0.3,1)){
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(),TARGET_5,0.3,1)){
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if(nav.driveTo(odo.getPosition(),TARGET_6,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if(nav.driveTo(odo.getPosition(),TARGET_7,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if(nav.driveTo(odo.getPosition(),TARGET_8,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if(nav.driveTo(odo.getPosition(),TARGET_9,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if(nav.driveTo(odo.getPosition(),TARGET_10,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if(nav.driveTo(odo.getPosition(),TARGET_11,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if(nav.driveTo(odo.getPosition(),TARGET_12,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_13;
                    }
                    break;
                case DRIVE_TO_TARGET_13:
                    if(nav.driveTo(odo.getPosition(),TARGET_13,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_14;
                    }
                    break;
                case DRIVE_TO_TARGET_14:
                    if(nav.driveTo(odo.getPosition(),TARGET_14,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_15;
                    }
                    break;
                case DRIVE_TO_TARGET_15:
                    if(nav.driveTo(odo.getPosition(),TARGET_15,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_16;
                    }
                    break;
                case DRIVE_TO_TARGET_16:
                    if(nav.driveTo(odo.getPosition(),TARGET_16,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_17;
                    }
                    break;
                case DRIVE_TO_TARGET_17:
                    if(nav.driveTo(odo.getPosition(),TARGET_17,0.3,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

//            telemetry.addData("Robot::Current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Robot::Position", data);

            telemetry.update();

        }
    }}
