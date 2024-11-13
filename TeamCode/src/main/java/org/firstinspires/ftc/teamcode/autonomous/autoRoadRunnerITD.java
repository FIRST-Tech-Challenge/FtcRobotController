package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.PIDFMotorController;

@Config
@Autonomous
public class autoRoadRunnerITD extends LinearOpMode {
    public class SpecClaw {
        private final Servo specServo;

        public SpecClaw(HardwareMap hardwareMap) {
            specServo = hardwareMap.get(Servo.class, "specServo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(0.2);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(0.8);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Slides {
        private final DcMotorEx rightSlideMotor;
        private final PIDFMotorController slideController;

        public Slides(HardwareMap hardwareMap) {
            rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
            double SLIDE_TICKS_IN_DEGREES = 537.7 / 360.0;
            slideController = new PIDFMotorController(rightSlideMotor, 0.01, 0.25, 0.001, 0, SLIDE_TICKS_IN_DEGREES);
        }

        public class SlidesUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.driveToPosition(2250);
                return false;
            }
        }

        public Action slidesUp() {
            return new SlidesUp();
        }

        public class SlidesDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.driveToPosition(0);
                return false;
            }
        }

        public Action slidesDown() {
            return new SlidesDown();
        }

        public class SlidesHold implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.driveToPosition(1750);
                return false;
            }
        }

        public Action slidesHold() {
            return new SlidesHold();
        }
    }

    public class IntakeArm {
        private final DcMotorEx intakeArmMotor;
        private final PIDFMotorController armController;

        public IntakeArm(HardwareMap hardwareMap) {
            intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
            double ARM_TICKS_IN_DEGREES = 1425.1 / 360.0;
            armController = new PIDFMotorController(intakeArmMotor, 0.01, 0.23, 0.001, 0.4, ARM_TICKS_IN_DEGREES);
        }

        public class IntakeArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armController.driveToPosition(50);
                return false;
            }
        }

        public Action intakeArmUp() {
            return new IntakeArmUp();
        }

        public class IntakeArmDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armController.driveToPosition(0);
                return false;
            }
        }


        public Action intakeArmDown() {
            return new IntakeArmDown();
        }
    }

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, -61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        SpecClaw specClaw = new SpecClaw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);

        TrajectoryActionBuilder firstSpec = drive.actionBuilder(beginPose) // go to sub, place spec
                .strafeTo(new Vector2d(0, -28));
        TrajectoryActionBuilder firstSpec1 = drive.actionBuilder(beginPose) // strafe to right
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, -34));
        TrajectoryActionBuilder firstSpec2 = drive.actionBuilder(beginPose) //push bot sample into human-player zone
                .strafeTo(new Vector2d(37, -35))
                .strafeTo(new Vector2d(37, -10))
                .splineTo(new Vector2d(47, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(47, -51))
                .strafeTo(new Vector2d(47, -59))
                .strafeTo(new Vector2d(47, -45))
                .waitSeconds(3)
                .strafeTo(new Vector2d(47, -61.5));
        TrajectoryActionBuilder secondSpec = drive.actionBuilder(beginPose)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(47, -45))
                .splineTo(new Vector2d(4, -52), Math.toRadians(270))
                .strafeTo(new Vector2d(4, -27));
        TrajectoryActionBuilder secondSpec1 = drive.actionBuilder(beginPose)
                        .waitSeconds(1)
                .splineTo(new Vector2d(47, -47), Math.toRadians(90))
                .strafeTo(new Vector2d(47, -61));
        Action trajectoryAction1 = firstSpec.build();
        Action trajectoryAction2 = firstSpec1.build();
        Action trajectoryAction3 = firstSpec2.build();
        Action trajectoryAction4 = secondSpec.build();
        Action trajectoryAction5 = secondSpec1.build();

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        specClaw.closeClaw(),
                        trajectoryAction1,
                        intakeArm.intakeArmDown(),
                        slides.slidesUp(),
                        slides.slidesHold(),
                        specClaw.openClaw(),
                        trajectoryAction2,
                        slides.slidesDown(),
                        trajectoryAction3,
                        specClaw.closeClaw(),
                        slides.slidesUp(),
                        trajectoryAction4,
                        slides.slidesHold(),
                        specClaw.openClaw(),
                        trajectoryAction5,
                        slides.slidesDown(),
                        intakeArm.intakeArmUp()
                )
        );
    }
}