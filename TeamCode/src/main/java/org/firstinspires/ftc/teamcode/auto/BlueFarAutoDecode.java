package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
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
import org.firstinspires.ftc.teamcode.archive.Hood;

@Config
public class BlueFarAutoDecode extends LinearOpMode {

    boolean isRed = false;
    @Autonomous()
    public static class BlueFar extends BlueFarAutoDecode {
        public BlueFar() {
            super(new Pose2d(60, -14.5, Math.toRadians(180)), new IdentityPoseMap());

        }
    }

    @Autonomous()
    @Disabled
    public static class RedFarAuto extends BlueFarAutoDecode {
        public RedFarAuto() {
            super(
                    new Pose2d(60, -14.5, Math.toRadians(180)
                    ),
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

    protected MecanumDrive drive;
    private DigitalChannel laserInput;
    DcMotor intake;
    DcMotor feeder;
    DcMotorEx shooter;
    DcMotorEx shooter2;
    Hood hood = new Hood();
    private Limelight3A limelight;
    private double shooterSpeed = 1075;

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
                double headingOffset = isRed ? Redoffset : Blueoffset;
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

    public BlueFarAutoDecode(Pose2d startingPose, PoseMap poseMap) {
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
        hood.setServoPos(0.4);
        double shooterSpeed = 1065;
        double currentShooterVelocity = shooter.getVelocity();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        Pose2d dumpPose1 = new Pose2d(7, -57, Math.toRadians(135));
        double dumpTangent1 = Math.toRadians(270);
        Pose2d shootPose = new Pose2d(-16, -16, Math.toRadians(135));
        Pose2d dumpPose2 = new Pose2d(25, -60, Math.toRadians(135));
        double dumpTangent2 = Math.toRadians(270);

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

                                // 1st Shot
                                .stopAndAdd(() -> artifactCounter = 3)
                                .strafeToLinearHeading(new Vector2d(56, -20), Math.toRadians(203))
                                .stopAndAdd(() -> artifactCounter = 0)

                                .stopAndAdd(() -> intake.setPower(1))
                                .waitSeconds(0.3)
                                .stopAndAdd(() -> feeder.setPower(1))
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feeder.setPower(0))
                                .stopAndAdd(() -> intake.setPower(0))
                                .stopAndAdd(() -> artifactCounter = 0)

                                // 1st Intake
                                .stopAndAdd(() -> intake.setPower(1))
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(31, -35, Math.toRadians(270)), Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(31, -53, Math.toRadians(265)), Math.toRadians(270))
                                .waitSeconds(0.6)
                                .stopAndAdd(() -> intake.setPower(0))

                                // 2nd Shot
                                .splineToLinearHeading(new Pose2d(56, -17, Math.toRadians(197)), Math.toRadians(90))
                                .stopAndAdd(() -> artifactCounter = 0)
                                .stopAndAdd(() -> intake.setPower(1))
                                .stopAndAdd(() -> feeder.setPower(1))
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feeder.setPower(0))
                                .stopAndAdd(() -> intake.setPower(0))
                                .stopAndAdd(() -> artifactCounter = 0)

                                // 2nd Intake
                                .stopAndAdd(() -> intake.setPower(1))
                                .strafeToLinearHeading(new Vector2d(48, -63), Math.toRadians(349))
                                .strafeToLinearHeading(new Vector2d(62, -64), Math.toRadians(349))
                                .waitSeconds(0.6)
                                .stopAndAdd(() -> intake.setPower(0))

                                // 3rd Shot
                                .strafeToLinearHeading(new Vector2d(56, -17), Math.toRadians(197))
                                .stopAndAdd(() -> artifactCounter = 0)
                                .stopAndAdd(() -> intake.setPower(1))
                                .stopAndAdd(() -> feeder.setPower(1))
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feeder.setPower(0))
                                .stopAndAdd(() -> intake.setPower(0))
                                .stopAndAdd(() -> artifactCounter = 0)

                                // 3rd Intake
                                .stopAndAdd(() -> intake.setPower(1))
                                .setTangent(Math.toRadians(270))
                                .strafeToSplineHeading(new Vector2d(61,-37.5),Math.toRadians(225))
                                .strafeToSplineHeading(new Vector2d(66,-58),Math.toRadians(225))
                                .strafeToLinearHeading(new Vector2d(33,-63), Math.toRadians(225))
                                .waitSeconds(0.6)
                                .stopAndAdd(() -> intake.setPower(0))

                                // 4th Shot
                                .strafeToLinearHeading(new Vector2d(56, -17), Math.toRadians(195))
                                .stopAndAdd(() -> artifactCounter = 0)
                                .stopAndAdd(() -> intake.setPower(1))
                                .stopAndAdd(() -> feeder.setPower(1))
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feeder.setPower(0))
                                .stopAndAdd(() -> intake.setPower(0)) // Turn off intake
                                .stopAndAdd(() -> artifactCounter = 0)

                                // 4th Intake
                                .stopAndAdd(() -> intake.setPower(1))
                                .setTangent(Math.toRadians(270))
                                .strafeToSplineHeading(new Vector2d(61,-37.5),Math.toRadians(225))
                                .strafeToSplineHeading(new Vector2d(66,-58),Math.toRadians(225))
                                .strafeToLinearHeading(new Vector2d(33,-63), Math.toRadians(225))
                                .waitSeconds(0.6)
                                .stopAndAdd(() -> intake.setPower(0))

                                // 5th Shot
                                .strafeToLinearHeading(new Vector2d(56, -17), Math.toRadians(195))
                                .stopAndAdd(() -> artifactCounter = 0)
                                .stopAndAdd(() -> intake.setPower(1))
                                .stopAndAdd(() -> feeder.setPower(1))
                                .waitSeconds(feederOnTime)
                                .stopAndAdd(() -> feeder.setPower(0))
                                .stopAndAdd(() -> intake.setPower(0)) // Turn off intake
                                .stopAndAdd(() -> artifactCounter = 0)

                                // 5th Intake
                                .stopAndAdd(() -> intake.setPower(1))
                                .setTangent(Math.toRadians(270))
                                .strafeToSplineHeading(new Vector2d(61,-37.5),Math.toRadians(283))
                                .strafeToSplineHeading(new Vector2d(66,-58),Math.toRadians(283))
                                .waitSeconds(0.6)
                                .stopAndAdd(() -> intake.setPower(0))

// region 6th Shot &  Intake
                                //                        // 6th shot
                                //                        .strafeToLinearHeading(new Vector2d(56, 20), Math.toRadians(157))
                                //                        .stopAndAdd(() -> artifactCounter = 0)
                                //                        .stopAndAdd(() -> intake.setPower(1))
                                //                        .stopAndAdd(() -> feeder.setPower(1))
                                //                        .waitSeconds(feederOnTime)
                                //                        .stopAndAdd(() -> feeder.setPower(0))
                                //                        .stopAndAdd(() -> intake.setPower(0)) // Turn off intake

                                //                        // 6th Intake
                                //                        .stopAndAdd(new RaceAction(intakeAction, AngleIntakingAction))
                                // endregion

                                // Park
                                .strafeToLinearHeading(new Vector2d(60, -40), Math.toRadians(283))

                                .build(),
                        intakeAction
                )
        );

        if(isStopRequested()) return;

        intake.setPower(0);
        feeder.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
        // }
        // private void pidTuner() {
        //   if (UPDATE_FLYWHEEL_PID) {
        //     PIDFCoefficients c = new PIDFCoefficients(TwentyTwentyFiveJava.FLYWHEEL_P, TwentyTwentyFiveJava.FLYWHEEL_I, TwentyTwentyFiveJava.FLYWHEEL_D, TwentyTwentyFiveJava.FLYWHEEL_F);
        //    shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        //    shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        //  UPDATE_FLYWHEEL_PID = false;
        //  }
        //telemetry.addData("Flywheel P", TwentyTwentyFiveJava.FLYWHEEL_P);
        //  telemetry.addData("Flywheel I", TwentyTwentyFiveJava.FLYWHEEL_I);
        // telemetry.addData("Flywheel D", TwentyTwentyFiveJava.FLYWHEEL_D);
        // telemetry.addData("Flywheel F", TwentyTwentyFiveJava.FLYWHEEL_F);

    }
}
