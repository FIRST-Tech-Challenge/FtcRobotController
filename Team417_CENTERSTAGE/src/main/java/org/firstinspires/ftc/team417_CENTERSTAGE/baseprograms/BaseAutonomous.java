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

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double APRIL_TAG_SLEEP_TIME = 500;
    public static double NO_APRIL_TAG_SLEEP_TIME = 2500;

    public int lastEncoderFL = 0;
    public int lastEncoderBL = 0;
    public int lastEncoderBR = 0;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 2; // in seconds

    public static double INTAKE_SPEED2 = 1;

    public static double INTAKE_TIME2 = 10; // in seconds

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;

    public OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);;

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        myColorDetection.init();
        initializeHardware();

        telemetry.addData("Init State", "Init Finished");

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

        /*The variable 'translateEnum' converts the OpenCV Enum by Hank to the Enum used by
        AutonDriveFactory created by Sawar.
        */
        AutonDriveFactory.SpikeMarks translateEnum;

        telemetry.addData("Side detected", result);
        telemetry.update();

        // Close cameras to avoid errors
        myColorDetection.robotCamera.closeCameraDevice();

        if (result == OpenCvColorDetection.SideDetected.LEFT) {
            translateEnum = AutonDriveFactory.SpikeMarks.LEFT;
        } else if (result == OpenCvColorDetection.SideDetected.CENTER) {
            translateEnum = AutonDriveFactory.SpikeMarks.CENTER;
        } else {
            translateEnum = AutonDriveFactory.SpikeMarks.RIGHT;
        }

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction poseAndAction = auton.getDriveAction(red, !close, translateEnum, dropPixel(1, 2));

        drive.pose = poseAndAction.startPose;

        if (!test) {
            Actions.runBlocking(poseAndAction.action);
        }

        if (drive.myAprilTagPoseEstimator != null) {
            drive.myAprilTagPoseEstimator.visionPortal.close();
        }
    }

    public void runAuto(boolean red, boolean close) {

        runAuto(red, close, false);
    }

    //Action: Spits out pixel in trajectory; see usage in AutonDriveFactory below.

    public Action dropPixel(double intakeTime, double intakeSpeed) {
        return new Action() {

            // Variable to store the start time for comparison
            double startTime = 0;
            @Override
            public boolean run(TelemetryPacket packet) {

                // Executes on the first loop to start the intake motor
                if (startTime == 0) {
                    intakeMotor.setPower(intakeSpeed);
                    startTime = nanoTime() * NANO_TO_SECONDS_MULTIPLIER;
                }
                // Checks if the elapsed time is greater than the intake time to stop the motor
                if(nanoTime() * NANO_TO_SECONDS_MULTIPLIER - startTime > intakeTime) {
                    intakeMotor.setPower(0);

                    // Resets start time for the next run
                    startTime = 0;
                    return false;
                } else {
                    return true;
                }
            }
        };
    }
}

class AutonDriveFactory {
    MecanumDrive drive;
    double xOffset;
    double yMultiplier;
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

   /* Booleans 'isRed' (red or blue side), 'isFar' (far or close to backdrop)
    'location' (center, middle, or right), and 'intake' (Action for use).
    */
    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake) {

        if (isFar) {
            xOffset = 0;
        } else {
            xOffset = 48;
        }

        if (isRed) {
            yMultiplier = 1;
        } else {
            yMultiplier = -1;
        }

        // in MeepMeep, intake needs to be null however .stopAndAdd() can't be null because it will crash so we set to a random sleep
        if(intake == null) {
            intake = new SleepAction(3);
        }

        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeLeft = spikeLeft.splineTo(xForm(new Vector2d(-34, -36)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-38, -34)), xForm(Math.toRadians(180) + (1e-6)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -33)), xForm(Math.toRadians(90)))
                // arm
                .splineToConstantHeading(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(-55, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(-55, -10)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));


        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeRight = spikeRight.splineToSplineHeading(xForm(new Pose2d(-35, -32, Math.toRadians(0))), xForm(Math.toRadians(90)))
                // arm action
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));

        if(location == SpikeMarks.LEFT) {
            return new PoseAndAction(spikeLeft.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else if(location == SpikeMarks.CENTER) {
            return new PoseAndAction(spikeCenter.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else {
            return new PoseAndAction(spikeRight.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        }

    }


    Pose2d xForm(Pose2d pose) {
        return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }


    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, true, SpikeMarks.LEFT, null).action;
    }
}
