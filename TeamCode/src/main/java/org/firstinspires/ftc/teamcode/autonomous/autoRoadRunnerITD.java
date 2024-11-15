package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.PIDFMotorController;

@Config
@Autonomous
public class autoRoadRunnerITD extends LinearOpMode {
    public static boolean USER_INPUT_FLAG = false;
    public static int SLIDES_ABOVE_BAR = 1900;
    public static int SLIDES_BELOW_BAR = 1270;
    public static int SLIDES_SPEC_PICKUP = 0;
    public static double SPEC_CLAW_CLOSE = 0.2;
    public static double SPEC_CLAW_OPEN = 0.8;
    public static int INTAKE_ARM_UP = 10;
    public static int INTAKE_ARM_DOWN = 250;
    public static double SLIDE_MAX_SPEED = 0.5;
    public static double ARM_MAX_SPEED = 0.5;

    public static class SpecClaw {
        private final Servo specServo;

        public SpecClaw(HardwareMap hardwareMap) {
            specServo = hardwareMap.get(Servo.class, "specServo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(SPEC_CLAW_CLOSE);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(SPEC_CLAW_OPEN);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public static class Slides {
        private final PIDFMotorController slideController;

        public Slides(HardwareMap hardwareMap) {
            DcMotorEx rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
            rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            double slideTicksInDegrees = 537.7 / 360.0;
            slideController = new PIDFMotorController(rightSlideMotor, 0.01, 0.25, 0.001, 0, slideTicksInDegrees, SLIDE_MAX_SPEED);
            slideController.resetMotorEncoder();
        }

        public class SlidesPIDIteration implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = slideController.runIteration();
                packet.put("slides motor power", power);
                return true;
            }
        }

        public Action slidesPIDIteration() {
            return new SlidesPIDIteration();
        }

        public class MoveSlidesAction implements Action {
            private final int targetPosition;

            public MoveSlidesAction(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.setTargetPosition(targetPosition);
                packet.put("slides target position", targetPosition);
                return false;
            }
        }

        public Action slidesToAboveBar() {
            return new MoveSlidesAction(SLIDES_ABOVE_BAR);
        }

        public Action slidesToBelowBar() {
            return new MoveSlidesAction(SLIDES_BELOW_BAR);
        }

        public Action slidesToSpecPickup() {
            return new MoveSlidesAction(SLIDES_SPEC_PICKUP);
        }
    }

    public static class IntakeArm {
        private final PIDFMotorController armController;

        public IntakeArm(HardwareMap hardwareMap) {
            DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
            intakeArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            double armTicksInDegrees = 1425.1 / 360.0;
            armController = new PIDFMotorController(intakeArmMotor, 0.01, 0.23, 0.001, 0.4, armTicksInDegrees, ARM_MAX_SPEED);
            armController.resetMotorEncoder();
        }

        public class ArmPIDIteration implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = armController.runIteration();
                packet.put("intake arm motor power", power);
                return true;
            }
        }

        public Action armPIDIteration() {
            return new ArmPIDIteration();
        }

        public class moveArmAction implements Action {
            private final int targetPosition;

            public moveArmAction(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armController.setTargetPosition(targetPosition);
                packet.put("intake arm target position", targetPosition);
                return false;
            }
        }

        public Action intakeArmUp() {
            return new moveArmAction(INTAKE_ARM_UP);
        }

        public Action intakeArmDown() {
            return new moveArmAction(INTAKE_ARM_DOWN);
        }
    }

    public static class WaitForUser implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(USER_INPUT_FLAG){
                USER_INPUT_FLAG = false;
                packet.put("Waiting For User", false);
                return false;
            }
            packet.put("Waiting For User", true);
            USER_INPUT_FLAG = false;
            return true;
        }
    }

    public Action waitForUser() {
        return new WaitForUser();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(13, -61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        SpecClaw specClaw = new SpecClaw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);

        TrajectoryActionBuilder moveAwayFromBarrier = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(13, -50));
        TrajectoryActionBuilder moveIntoSpec1Position = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -30));
        TrajectoryActionBuilder driveBack = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -34));
        TrajectoryActionBuilder pushSampleGrabSpec = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(37, -35))
                .strafeTo(new Vector2d(37, -10))
                .splineTo(new Vector2d(47, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(47, -51))
                .strafeTo(new Vector2d(47, -59))
                .strafeTo(new Vector2d(47, -45))
                .waitSeconds(3)
                .strafeTo(new Vector2d(47, -61.5));
        TrajectoryActionBuilder goToSubSecondSpec = drive.actionBuilder(beginPose)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(47, -45))
                .splineTo(new Vector2d(4, -52), Math.toRadians(270))
                .strafeTo(new Vector2d(4, -27));
        TrajectoryActionBuilder goBackAndPark = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .splineTo(new Vector2d(47, -47), Math.toRadians(90))
                .strafeTo(new Vector2d(47, -58));

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action moveIntoSpec1PositionAction = moveIntoSpec1Position.build();
        Action driveBackAction = driveBack.build();
        Action pushSampleGrabSpecAction = pushSampleGrabSpec.build();
        Action goToSubSecondSpecAction = goToSubSecondSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                specClaw.closeClaw(), // Hold onto Spec
                waitForUser(),
                moveAwayFromBarrierAction, // Move away from the barrier
                waitForUser(),
                intakeArm.intakeArmDown(), // Move the intake arm out of the way of the slides
                waitForUser(),
                slides.slidesToAboveBar(), // Move the slides to above the bar to prepare for the first spec
                waitForUser(),
                moveIntoSpec1PositionAction, // Move into position to place the first spec
                waitForUser(),
                slides.slidesToBelowBar(), // Clip the spec onto the bar
                waitForUser(),
                specClaw.openClaw(), // Release the spec
                waitForUser(),
                driveBackAction, //drive back to put slides fully down
                waitForUser(),
                slides.slidesToSpecPickup(), //brings slides to pos 0 (fully down)
                waitForUser(),
                pushSampleGrabSpecAction, //pushes sample into player person zone, then grabs spec
                waitForUser(),
                specClaw.closeClaw(), // Hold onto Spec
                waitForUser(),
                slides.slidesToAboveBar(), //bring slides up to Spec Position
                waitForUser(),
                goToSubSecondSpecAction, //go to sub to put on second spec
                waitForUser(),
                slides.slidesToBelowBar(), //clip on spec
                waitForUser(),
                specClaw.openClaw(), // Release the spec
                waitForUser(),
                goBackAndParkAction, //park in player person zone
                waitForUser(),
                slides.slidesToSpecPickup(), //bring slides down
                waitForUser(),
                new SleepAction(3),
                intakeArm.intakeArmUp() //bring intake arm in to get ready for teleop
        );
        Action pidControlLoops = new ParallelAction(
                slides.slidesPIDIteration(), //cycling through slides to hold PID
                intakeArm.armPIDIteration() //cycling through intakeArm to hold PID
        );

        waitForStart();
        Actions.runBlocking(new ParallelAction(autoSequence, pidControlLoops));
    }
}