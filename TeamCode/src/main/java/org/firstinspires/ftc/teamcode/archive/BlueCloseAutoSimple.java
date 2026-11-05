package org.firstinspires.ftc.teamcode.archive;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class BlueCloseAutoSimple extends LinearOpMode {

    boolean isBlue = false;
    @Disabled
    @Autonomous()
    public static class BlueCloseSimple extends BlueCloseAutoSimple {
        public BlueCloseSimple() {
            super(new Pose2d(-60, -37, Math.toRadians(0)), new IdentityPoseMap());
            isBlue = true;
        }
    }

/*
    @Autonomous()
    public static class BlueClose18BallDumpOnly extends Close18BallDumpOnlyBlue {
        public BlueClose18BallDumpOnly() {
            super(
                    new Pose2d(60, -37, Math.toRadians(0)),
                    pose -> new Pose2dDual<>(
                            new Vector2dDual<>(
                                    pose.position.x,
                                    pose.position.y.unaryMinus()
                            ),
                            pose.heading.inverse()
                    )
            );
        }
    }
*/

    protected MecanumDrive drive;
    private DigitalChannel laserInput;
    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    Hood hood = new Hood();
    private Limelight3A limelight;
    private double shooterSpeed;

    public static boolean
            UPDATE_FLYWHEEL_PID = true;
    public static boolean artifactPresent = false;

    public static int artifactCounter = 0;
    public static double Redoffset = 0;
    public static double Blueoffset = 0;
    public static double BEARING_THRESHOLD = 0.25; // Angled towards the tag (degrees)
    public static double TURN_GAIN   =  0.04  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double TURN_STATIC = 0.1;
    public static double MAX_AUTO_TURN  = 0.3;
    public static double feederOnTime = 0.6;
    PoseMap poseMap;
    Pose2d startingPose;

    Action countArtifacts() {
        return new Action() {
            long startTimeMillis = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                //

                return (now - startTimeMillis < 600);

            }
        };
    }
    Action autoAimAction() {
        return new Action() {

            long startTimeMillis = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                // Do your auto aim code here
                double turn = 0;

                LLResult llResult = limelight.getLatestResult();
                double headingError = llResult.getTx();
                double headingOffset = isBlue ? Blueoffset : Redoffset;
                double offsetError = headingError + headingOffset;
                if (Math.abs(offsetError) < BEARING_THRESHOLD) {
                    return false;
                }
                turn = Range.clip((offsetError + (Math.signum(offsetError) * TURN_STATIC)) * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn * -1));

                return (now - startTimeMillis < 2_000);
            }
        };
    }

    Action shooterCheckAction() {
        return new Action() {

            long startTimeMillis = 0;
            double currentShooterVelocity;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();
                if (startTimeMillis == 0) {
                    startTimeMillis = now;
                }

                currentShooterVelocity = shooter.getVelocity();
                telemetryPacket.put("Current Velocity", currentShooterVelocity);
                telemetryPacket.put("Target Velocity", shooterSpeed);
                if (Math.abs(currentShooterVelocity - shooterSpeed) < 50  ) {
                    return false;
                }

                return (now - startTimeMillis < 1_000);

            }
        };
    }

    public BlueCloseAutoSimple(Pose2d startingPose, PoseMap poseMap) {
        this.poseMap = poseMap;
        this.startingPose = startingPose;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        laserInput = hardwareMap.get(DigitalChannel.class, "LaserArtifactDetector");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        intake = hardwareMap.get(DcMotor.class, "Intake");
        feeder = hardwareMap.get(DcMotor.class, "Shooter Feeder");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        hood.init(hardwareMap);
        hood.setServoPos(0.44);
        double shooterSpeed = 767;
        double currentShooterVelocity = shooter.getVelocity();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        Pose2d dumpPose1 = new Pose2d(5, -68, Math.toRadians(225));
        double dumpTangent1 = Math.toRadians(270);
        Pose2d shootPose = new Pose2d(-16, -16, Math.toRadians(217));
        Pose2d dumpPose2 = new Pose2d(20, -63, Math.toRadians(225));
        double dumpTangent2 = Math.toRadians(0);

//        PIDFCoefficients c = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        if (!UPDATE_FLYWHEEL_PID) {
//            TwentyTwentyFiveJava.FLYWHEEL_P = c.p;
//         TwentyTwentyFiveJava.FLYWHEEL_I = c.i;
//            TwentyTwentyFiveJava.FLYWHEEL_D = c.d;
//            TwentyTwentyFiveJava.FLYWHEEL_F = c.f;
//        }
//        pidTuner();
        PIDFCoefficients pidfNew = new PIDFCoefficients (200, 0, 0, 22.16);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfNew);

        Pose2d beginPose = startingPose;// new Pose2d(60, 30, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, beginPose);
        limelight.start();
        telemetry.addData("Current Velocity", 0);
        telemetry.addData("Target Velocity", 0);
        telemetry.addData("Feeder Speed", 0);
        waitForStart();
        PIDFCoefficients currentPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel P", currentPIDF.p);
        telemetry.addData("Flywheel F", currentPIDF.f);

        Action intakeAction = (telemetryPacket) -> {
            // Artifact Counter Logic
            boolean artifactDetected = laserInput.getState();
            if (artifactDetected && !artifactPresent) {
                artifactCounter++;
                artifactPresent = true;
            }
            if (!artifactDetected) {
                artifactPresent = false;
            }
            if (artifactCounter < 3) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // Add to RoadRunner Dashboard/Telemetry
            telemetryPacket.put("Artifact counter", artifactCounter);
            telemetry.addData("Artifact counter", artifactCounter);
            telemetry.update();

            return true;
        };

        Actions.runBlocking(
                new RaceAction(
                        drive.actionBuilder(beginPose, poseMap)

                                // Turn on motors
                                .stopAndAdd(() -> {
                                    intake.setPower(0.0);
                                    feeder.setPower(0.0);
                                    shooter.setVelocity(shooterSpeed);
                                    shooter2.setVelocity(shooterSpeed);
                                })

                                //First Shot
                                .stopAndAdd(() -> artifactCounter = 3)
                                .strafeToLinearHeading(new Vector2d(-16, -16), Math.toRadians(217))
                                .waitSeconds(2)
                                .stopAndAdd(() -> artifactCounter = 0)
                                .stopAndAdd(() -> intake.setPower(1))
                                .stopAndAdd(() -> feeder.setPower(1))
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feeder.setPower(0))
                                .stopAndAdd(() -> intake.setPower(0))
                                .stopAndAdd(() -> artifactCounter = 0)

                                //Park
                                .strafeToLinearHeading(new Vector2d(-60, -16), Math.toRadians(270))
                                .build(),
                        intakeAction
                )
        );

        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
    }
}
