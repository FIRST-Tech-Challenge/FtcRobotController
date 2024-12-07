package org.firstinspires.ftc.teamcode.opmodes;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Config
@Autonomous(name = "RedLeftBasketThenPark")
public class RedLeftBasketThenPark extends LinearOpMode {
    private boolean first = true;
    private static final double FIRST_LIFT_DOWN_POS = 50.0;
    private static final double LAST_LIFT_DOWN_POS = 100.0;
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
        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)
                .lineToY(-52)
                .turn(Math.toRadians(90))
                .lineToX(-52)
                .turn(Math.toRadians(45))

// to submersible
//                .lineToY(-1)
  //              .waitSeconds(2)
    //            .turn(Math.toRadians(-96.5))
      //          .lineToX(-23)


//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
                 .waitSeconds(3);

        // ON INIT:
  //      Actions.runBlocking(claw.closeClaw());

        Action firstTraj = toSub.build();

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
  //                      lift.liftUp(),
    //                    claw.openClaw(),
                        firstTraj // go to the basket and then submersible
                        //lift.liftUp() // to lvl1 ascent
                      //  claw.openClaw(), // drop the sample
                      //  lift.liftDown()
                )
        );
    }

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }
                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
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
        }
        public Action liftUp() {
            return new LiftUp();
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
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.2);
                return false;
            }
        }

        public Action openClaw() {
            return new CloseClaw();
        }
    }

    public class LiftPivot {
        private DcMotorEx liftPivot;

        public LiftPivot(HardwareMap hardwareMap) {
            liftPivot = hardwareMap.get(DcMotorEx.class, "liftPivot");
            liftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
            liftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftPivot.setTargetPosition(900);
            liftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class LiftPivotUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftPivot.setPower(0.8);
                    initialized = true;
                }
                // checks lift's current position
                double pos = liftPivot.getCurrentPosition();
                packet.put("liftPivotPos", pos);
                if (pos < 500.0) {
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

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftPivot.setPower(0.8);
                    initialized = true;
                }
                if (first) {
                    currLiftPos = FIRST_LIFT_DOWN_POS;
                    first = false;
                } else {
                    currLiftPos = LAST_LIFT_DOWN_POS;
                }
                double pos = liftPivot.getCurrentPosition();
               // packet.put("liftPos", pos);
                telemetry.addData("liftPivotPos",pos);
                telemetry.update();

                if (pos < currLiftPos) {
                    return true;
                } else {
                    liftPivot.setPower(0);
                    return false;
                }
            }
        }

        public Action liftPivotDown() {
            return new LiftPivotDown();
        }


    }

}

