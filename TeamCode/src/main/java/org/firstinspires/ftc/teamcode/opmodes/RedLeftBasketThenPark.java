package org.firstinspires.ftc.teamcode.opmodes;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Config
@Autonomous(name = "RedLeftBasketThenPark")
public class RedLeftBasketThenPark extends LinearOpMode {
    private boolean first = true;
    private double currLiftPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiating the robot at a specific pose
        Pose2d initialPose = new Pose2d(-38, -62, Math.toRadians(89));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        LiftPivot liftPivot = new LiftPivot(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBasket = drive.actionBuilder(initialPose)
                .lineToY(-52)
                .turn(Math.toRadians(90))
                .lineToX(-58)
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-62,-55))
                .waitSeconds(1.5);

        Action toSub = toBasket.endTrajectory().fresh()
                // samples (push)
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-45,-55))
                .strafeTo(new Vector2d(-45,-15))
                .turn(Math.toRadians(90))
                .lineToX(-26)
                .build();


        // ON INIT:
  //      Actions.runBlocking(claw.closeClaw());

        Action firstTraj = toBasket.build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        // IN RUNTIME
        // running the action sequence!
        Actions.runBlocking(
                new SequentialAction(
//                        liftPivot.liftPivotDown(),
                        firstTraj, // go to the basket, push samples, and then submersible
                        liftPivot.liftPivotUp(),
                        lift.liftUp(),
                        claw.openClaw(), // drop the sample
                        lift.liftDown(),
                        toSub, // push samples, go to submersible
                        liftPivot.liftPivotUp(),
                        lift.liftUpLittle(),
                        liftPivot.liftPivotDown()
                )
        );
    }

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(1);
                    initialized = true;
                }
                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPivotPos", pos);
                telemetry.addData("Lift pivot pos: ", pos);
                if (pos < 3000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off

        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftUpLittle implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(1);
                    initialized = true;
                }
                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPivotPos", pos);
                telemetry.addData("Lift pivot pos: ", pos);
                if (pos < 500.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off

        }
        public Action liftUpLittle() {
            return new LiftUpLittle();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

    }

    public class Claw {
        private CRServo claw;
        private CRServo claw2;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(CRServo.class, "claw");
            claw2 = hardwareMap.get(CRServo.class, "claw2");
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPower(1);
                claw2.setPower(-1);
                sleep(2000);
                claw.setPower(0);
                claw2.setPower(0);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPower(-1);
                claw2.setPower(1);
                sleep(2000);
                claw.setPower(0);
                claw2.setPower(0);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class LiftPivot {
        private DcMotorEx liftPivot;

        public LiftPivot(HardwareMap hardwareMap) {
            liftPivot = hardwareMap.get(DcMotorEx.class, "liftPivot");
            liftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftPivot.setDirection(DcMotorSimple.Direction.REVERSE);

            liftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        public class LiftPivotUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftPivot.setPower(1);
                    initialized = true;
                }
                // checks lift's current position
                double pos = liftPivot.getCurrentPosition();
                packet.put("liftPivotPos", pos);
                if (pos < 1690.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    liftPivot.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action liftPivotUp() {
            return new LiftPivotUp();
        }

        public class LiftPivotDown implements Action {
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftPivot.setPower(-1);
                    initialized = true;
                }
                // checks lift's current position
                double pos = liftPivot.getCurrentPosition();
                packet.put("liftPivotPos", pos);
                if (pos > 500.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    liftPivot.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftPivotDown() {
            return new LiftPivotDown();
        }


    }

}

