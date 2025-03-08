package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "BlueTest", group = "Autonomous")
public class BlueTest extends LinearOpMode {

    public class Lift {
        private DcMotorEx liftMotor1, liftMotor2;

        public Lift(HardwareMap hardwareMap) {
            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

            liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private void moveLiftToPosition(int targetPosition, double power) {
            liftMotor1.setTargetPosition(targetPosition);
            liftMotor2.setTargetPosition(targetPosition);

            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor1.setPower(power);
            liftMotor2.setPower(power);

            while (liftMotor1.isBusy() && liftMotor2.isBusy() && opModeIsActive()) {
                telemetry.addData("LiftMotor1 Position", liftMotor1.getCurrentPosition());
                telemetry.addData("LiftMotor2 Position", liftMotor2.getCurrentPosition());
                telemetry.update();
            }

            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public Action moveLiftAction(int targetPosition, double power) {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        moveLiftToPosition(targetPosition, power);
                        initialized = true;
                    }
                    return false;
                }
            };
        }
    }

    public class RobotServos {
        private Servo rotateArm;
        private Servo bottomClaw;
        private Servo rotateBClaw;
        private Servo flipTClaw;
        private Servo rotateTClaw;
        private Servo topClaw;

        public RobotServos(HardwareMap hardwareMap) {
            rotateArm    = hardwareMap.get(Servo.class, "rotateArm");
            bottomClaw   = hardwareMap.get(Servo.class, "bottomClaw");
            rotateBClaw  = hardwareMap.get(Servo.class, "rotateBClaw");
            flipTClaw    = hardwareMap.get(Servo.class, "flipTClaw");
            rotateTClaw  = hardwareMap.get(Servo.class, "rotateTClaw");
            topClaw      = hardwareMap.get(Servo.class, "topClaw");
        }


        public Action setServoPosition(Servo servo, double targetPos) {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        servo.setPosition(targetPos);
                        initialized = true;
                    }
                    return false;
                }
            };
        }

        public Action moveBottomClaw(double position) {
            return setServoPosition(bottomClaw, position);
        }
        public Action moveTopClaw(double position) {
            return setServoPosition(topClaw, position);
        }
        public Action moveRotateArm(double position) {
            return setServoPosition(rotateArm, position);
        }
        public Action moveRotateBClaw(double position) {
            return setServoPosition(rotateBClaw, position);
        }
        public Action moveFlipTClaw(double position) {
            return setServoPosition(flipTClaw, position);
        }
        public Action moveRotateTClaw(double position) {
            return setServoPosition(rotateTClaw, position);
        }
    }

    //-------------------------------------------------------------------------
    // MAIN AUTONOMOUS CODE
    //-------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        // 1) INITIALIZE DRIVE (frontLeft, rearLeft, frontRight, rearRight)
        //    and set the robot's starting pose
        double halfWidth = 7.4375;
        double halfLength = 8.125;
        Pose2d initialPose = new Pose2d(24 - halfWidth, -72 + halfLength, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        // 2) INITIALIZE LIFT & CLAW
        Lift lift = new Lift(hardwareMap);
        RobotServos servos  = new RobotServos(hardwareMap);
        Servo rotateArm    = hardwareMap.get(Servo.class, "rotateArm");

        int liftMotor1StartPosition = lift.liftMotor1.getCurrentPosition();
        int specimenHangingPosition = liftMotor1StartPosition + 3000;

        Action liftToHangSpecimen = lift.moveLiftAction(3000, 0.8);
        Action openBottomClaw  = servos.moveBottomClaw(0.0);
        Action closeBottomClaw = servos.moveBottomClaw(1.0);
        Action rotateArmOut    = servos.moveRotateArm(1.0);
        Action flipTClawOut = servos.moveFlipTClaw(0); // for specimen
        Action rotateTClaw = servos.moveRotateTClaw(1); // for specimen

        // 4) BUILD YOUR TRAJECTORIES
        //    This is purely an example that you can revise for your route
        TrajectoryActionBuilder drive1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12 + halfWidth, -39.5 + halfLength))
                /*.waitSeconds(1)
                .lineToY(-42.5 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(47 - halfWidth, -45.5 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(53, -22 + halfLength),null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52, null, new ProfileAccelConstraint(-80, 80))
                .lineToY(-22 + halfLength, null, new ProfileAccelConstraint(-80, 80))
                .strafeTo(new Vector2d(63, -22 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToY(-52)
                .strafeTo(new Vector2d(64,-42), null, new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52, -76 + halfLength, Math.toRadians(250)),Math.toRadians(250), null, new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1)
                .strafeTo(new Vector2d(42, -65 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(-6, -39.5 + halfLength, Math.toRadians(90)),Math.toRadians(90), null, new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1)
                .strafeTo(new Vector2d(5, -45 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(45, -76 + halfLength, Math.toRadians(250)),Math.toRadians(250), null, new ProfileAccelConstraint(-80, 80))
                .waitSeconds(1)
                .strafeTo(new Vector2d(35, -65 + halfLength), null, new ProfileAccelConstraint(-80, 80))
                .splineToLinearHeading(new Pose2d(-6, -39.5 + halfLength, Math.toRadians(90)),Math.toRadians(90), null, new ProfileAccelConstraint(-80, 80))*/
                ;

        // 5) (OPTIONAL) DO ANY ACTIONS ON INIT
        //Actions.runBlocking(claw.closeClaw()); // For instance, keep claw closed

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("Autonomous", "Started");
        rotateArm.setPosition(1.0);
        telemetry.addData("Autonomous", "Complete");

        Action trajectoryAction1 = drive1.build();



        // 8) RUN THE ACTION SEQUENCE
        /*Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1,
                        liftToHangSpecimen,
                        flipTClawOut,
                        rotateTClaw
                )
        ); */
    }
}




