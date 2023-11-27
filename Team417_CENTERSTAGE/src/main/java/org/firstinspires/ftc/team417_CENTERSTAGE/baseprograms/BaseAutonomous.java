package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import static java.lang.System.nanoTime;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.apriltags.AprilTagPoseEstimator;
import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public int lastEncoderFL = 0;
    public int lastEncoderFR = 0;
    public int lastEncoderBL = 0;
    public int lastEncoderBR = 0;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 2; // in seconds

    public static double INTAKE_SPEED2 = 1;

    public static double INTAKE_TIME2 = 10; // in seconds

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;

    MecanumDrive drive;

<<<<<<< HEAD
        double targetFL = xTicks + yTicks;
        double targetFR = yTicks - xTicks;
        double targetBL = yTicks - xTicks;
        double targetBR = yTicks + xTicks;

        // Determine new target position, and pass to motor controller
        targetFL += FL.getCurrentPosition();
        targetFR += FR.getCurrentPosition();
        targetBL += BL.getCurrentPosition();
        targetBR += BR.getCurrentPosition();

        FL.setTargetPosition((int) targetFL);
        FR.setTargetPosition((int) targetFR);
        BL.setTargetPosition((int) targetBL);
        BR.setTargetPosition((int) targetBR);

        // Turn On RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        FL.setPower(ROBOT_SPEED);
        FR.setPower(ROBOT_SPEED);
        BL.setPower(ROBOT_SPEED);
        BR.setPower(ROBOT_SPEED);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {

            // Code added to draw the pose, remove before competition, causes lags:
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();
            myAprilTagPoseEstimator.updatePoseEstimate();
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);
        }

        // Stop all motion;
        stopDriving();

        // Turn off RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
=======
>>>>>>> cf7e29e2c244efe4c2bc89e2206c768c25fa7b31

    public AprilTagPoseEstimator myAprilTagPoseEstimator = new AprilTagPoseEstimator(this);
    public OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    public void initializeAuto() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        myColorDetection.init();
        myAprilTagPoseEstimator.init();
        initializeHardware();

        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = FR.getCurrentPosition();
        lastEncoderFL = FL.getCurrentPosition();
        lastEncoderBL = BL.getCurrentPosition();
        lastEncoderBR = BR.getCurrentPosition();

        // Allow the OpenCV to process
        sleep(500);

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    public void runAuto(boolean red, boolean close) {

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

        if (result == OpenCvColorDetection.SideDetected.LEFT) {
            sawarResult = AutonDriveFactory.SpikeMarks.LEFT;
        } else if (result == OpenCvColorDetection.SideDetected.CENTER) {
            sawarResult = AutonDriveFactory.SpikeMarks.CENTER;
        } else {
            sawarResult = AutonDriveFactory.SpikeMarks.RIGHT;
        }

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction poseAndAction = auton.getDriveAction(red,!close, sawarResult, dropPixel());

        drive.pose = poseAndAction.startPose;
        Actions.runBlocking(poseAndAction.action);
    }

    public Action dropPixel() {
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
