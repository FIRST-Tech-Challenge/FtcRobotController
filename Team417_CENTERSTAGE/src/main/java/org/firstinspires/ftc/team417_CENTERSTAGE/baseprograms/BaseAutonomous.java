package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import java.util.concurrent.CompletableFuture;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double APRIL_TAG_SLEEP_TIME = 500;
    public static double NO_APRIL_TAG_SLEEP_TIME = 5000;

    public int lastEncoderFL = 0;
    public int lastEncoderFR = 0;
    public int lastEncoderBL = 0;
    public int lastEncoderBR = 0;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 2; // in seconds

    public static double INTAKE_SPEED2 = 0.2;

    public static double INTAKE_TIME2 = 10; // in seconds

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;


    public static double PARKING_OFFSET = -6;
    public static double PARKING_ANGLE = -80;
    public static double PARKING_Y_OFFSET = 4;

    MecanumDrive drive;

    public OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);;

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        myColorDetection.init();
        initializeHardware();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = FR.getCurrentPosition();
        lastEncoderFL = FL.getCurrentPosition();
        lastEncoderBL = BL.getCurrentPosition();
        lastEncoderBR = BR.getCurrentPosition();

        // Allow the OpenCV to process
        if (drive.USE_APRIL_TAGS) {
            sleep((long) APRIL_TAG_SLEEP_TIME);
        } else {
            sleep((long) NO_APRIL_TAG_SLEEP_TIME);
        }

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    public void runAuto(boolean red, boolean close, boolean test) {
        if (red) {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
            telemetry.addLine("Looking for red");
        } else {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
            telemetry.addLine("Looking for blue");
        }

        initializeAuto();

        waitForStart();

        OpenCvColorDetection.SideDetected result = myColorDetection.detectTeamProp();
        AutonDriveFactory.SpikeMarks sawarResult;

        telemetry.addData("Side detected", result);
        telemetry.update();

        // Close cameras to avoid errors
        myColorDetection.robotCamera.closeCameraDevice();

        if (result == OpenCvColorDetection.SideDetected.LEFT) {
            sawarResult = AutonDriveFactory.SpikeMarks.LEFT;
        } else if (result == OpenCvColorDetection.SideDetected.CENTER) {
            sawarResult = AutonDriveFactory.SpikeMarks.CENTER;
        } else {
            sawarResult = AutonDriveFactory.SpikeMarks.RIGHT;
        }

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction poseAndAction = auton.getDriveAction(red, !close, sawarResult, dropPixelWithIntake(), placePixelWithArm());

        drive.pose = poseAndAction.startPose;

        if (!test) {
            CompletableFuture<Void> future = CompletableFuture.runAsync(() -> Actions.runBlocking(poseAndAction.action));

            while (opModeIsActive());

            future.cancel(true);
        } else {
            while (opModeIsActive());
        }


        //if (drive.myAprilTagPoseEstimator != null) {
        //    drive.myAprilTagPoseEstimator.visionPortal.close();
        //}
    }

    public void runAuto(boolean red, boolean close) {
        runAuto(red, close, false);
    }

    public Action dropPixelWithIntake() {
        return new Action() {
            double startTime = 0;  // startTime value to compare to
            @Override
            public boolean run(TelemetryPacket packet) {
                if (startTime == 0) { // does this on first loop
                    intakeMotor.setPower(INTAKE_SPEED2);
                    startTime = nanoTime() * NANO_TO_SECONDS_MULTIPLIER;
                }
                // current time - start time has to be greater than the intake time for the motor to stop
                if(nanoTime() * NANO_TO_SECONDS_MULTIPLIER - startTime > INTAKE_TIME) {
                    intakeMotor.setPower(0);
                    startTime = 0; // reset for next run
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    public Action placePixelWithArm() {
        return new Action() {
            //double startTime = 0;  // startTime value to compare to
            @Override
            public boolean run(TelemetryPacket packet) {
                final double startUpwardTiltPos = 75, startUpwardResetPos = 1500, endUpwardResetPos = 2000, startDownwardTiltPos = 1500, endDownwardTiltPos = 250; //Locations the dumper should tilt so the arm does not hit the robot.
                if (armMotor.getCurrentPosition() <= BaseOpMode.ARM_MOTOR_MAX_POSITION * (3.0/4.0)) {
                    armMotor.setPower(0.4);
                    if ((armMotor.getCurrentPosition() > startUpwardTiltPos && armMotor.getCurrentPosition() < startUpwardResetPos) && armMotor.getPower() > 0)
                        dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);
                    else if (armMotor.getCurrentPosition() < endUpwardResetPos && armMotor.getPower() > 0)
                        dumperServo.setPosition(DUMPER_SERVO_RESET_POSITION);

                    if (armMotor.getCurrentPosition() < endDownwardTiltPos && armMotor.getPower() < 0)
                        dumperServo.setPosition(DUMPER_SERVO_RESET_POSITION);
                    else if (armMotor.getCurrentPosition() < startDownwardTiltPos && armMotor.getPower() < 0)
                        dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);
                    return true;
                } else {
                    armMotor.setPower(0);
                    tiltDumper();
                    openGate();
                    return false;
                }
            }
        };
    }
}

class AutonDriveFactory {
    MecanumDrive drive;
    double xOffset;
    double yMultiplier;

    double parkingOffset;

    double parkingOffsetCenterFar;

    double centerMultiplier;

    double centerOffset;



    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    enum SpikeMarks {
        LEFT,
        CENTER,
        RIGHT
    }

    class PoseAndAction {
        Action action;
        Pose2d startPose;

        PoseAndAction(Action action, Pose2d startPose) {
            this.action = action;
            this.startPose = startPose;
        }
    }

    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake, Action output) {
  
        if (isFar) {
            xOffset = 0;
            parkingOffset = 48;
            centerMultiplier = 1;
            centerOffset = 0;

            if (location == xForm(SpikeMarks.CENTER)) {
                parkingOffset = 100;
            }


        } else {
            xOffset = 48;
            parkingOffset = 2;
            centerMultiplier = -1;
            centerOffset = 96;

        }

        if (isRed) {
            yMultiplier = 1;
        } else {
            yMultiplier = -1;
        }

        // in MeepMeep, intake needs to be null however .stopAndAdd() can't be null because it will crash so we set to a random sleep
        if (intake == null) {
            intake = new SleepAction(3);
        }

        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeLeft = spikeLeft.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-35, -34)), xForm((Math.toRadians(180) + (1e-6))))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset + BaseAutonomous.PARKING_OFFSET, -12)), xForm(Math.toRadians(0))) //drive to the backdrop
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(-90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset + BaseAutonomous.PARKING_OFFSET, -36 + BaseAutonomous.PARKING_Y_OFFSET)), xForm(Math.toRadians(BaseAutonomous.PARKING_ANGLE)))
                .stopAndAdd(output);

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, (Math.toRadians(90)))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xFormCenter(new Vector2d(-55, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xFormCenter(new Vector2d(-55, -30)), xForm(Math.toRadians(90)))
                .splineTo(xFormCenter(new Vector2d(parkingOffset - 43, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset + BaseAutonomous.PARKING_OFFSET, -12)), xForm(Math.toRadians(0))) //drive to the backdrop
                .turn(xForm(Math.toRadians(180))) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(-90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset + BaseAutonomous.PARKING_OFFSET, -36)), xForm(Math.toRadians(BaseAutonomous.PARKING_ANGLE)))
                .stopAndAdd(output);


        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeRight = spikeRight.splineTo(xForm(new Vector2d(-35, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-33, -37)), xForm(Math.toRadians(0)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset + BaseAutonomous.PARKING_OFFSET, -12)), xForm(Math.toRadians(0))) //drive to the backdrop
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(-90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset + BaseAutonomous.PARKING_OFFSET, -36 - BaseAutonomous.PARKING_Y_OFFSET)), xForm(Math.toRadians(BaseAutonomous.PARKING_ANGLE)))
                .stopAndAdd(output);

        if (location == xForm(SpikeMarks.LEFT)) {
            return new PoseAndAction(spikeLeft.build(), xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        } else if (location == xForm(SpikeMarks.RIGHT)) {
            return new PoseAndAction(spikeRight.build(), xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        } else {
            return new PoseAndAction(spikeCenter.build(), xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        }

    }


    Pose2d xForm(Pose2d pose) {
        return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Pose2d xFormCenter(Pose2d pose) {
        return new Pose2d((pose.position.x + centerOffset), pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    Vector2d xFormCenter(Vector2d vector) {
        return new Vector2d((vector.x + centerOffset), vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }


    SpikeMarks xForm(SpikeMarks spike) {
        if (yMultiplier == -1) {
            switch (spike) {
                case LEFT:
                    return SpikeMarks.RIGHT;
                case RIGHT:
                    return SpikeMarks.LEFT;
            }
        }
        return spike;
    }
}



