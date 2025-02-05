package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_FSM", group = "Autonomous")
@Disabled
public class Test extends LinearOpMode {

    // Define all the robot components
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void liftUp() {
            lift.setPower(0.8);  // Adjust power to raise the arm
        }

        public void liftDown() {
            lift.setPower(-0.8); // Adjust power to lower the arm
        }

        public void stopLift() {
            lift.setPower(0);
        }

        public int getLiftPosition() {
            return lift.getCurrentPosition();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public Action closeClaw() {
            claw.setPosition(0.55); // Adjust to close claw
            return null;
        }

        public void openClaw() {
            claw.setPosition(1.0); // Adjust to open claw
        }
    }

    // Define states
    private enum State {
        IDLE,
        MOVING_TO_POSITION,
        STRAFE_TO_POSITION,
        LIFT_UP,
        OPEN_CLAW,
        CLOSE_CLAW,
        STRAFE_BACK_TO_POSITION,
        LOWER_ARM,
        SPLINE_TO_POSITION,
        PICK_UP_GAMEPIECE,
        COMPLETE
    }

    @Override
    public void runOpMode() {
        // Initialize robot components
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90)); // Start at (0,0)
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // Initialize state machine
        State currentState = State.IDLE;

        // Run initialization actions
        Actions.runBlocking(claw.closeClaw());

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            switch (currentState) {
                case IDLE:
                    // Initial state, wait until the start button is pressed
                    telemetry.addData("Status", "Idle");
                    telemetry.update();
                    currentState = State.MOVING_TO_POSITION;
                    break;

                case MOVING_TO_POSITION:
                    telemetry.addData("State", "Moving to (50, 10)");
                    telemetry.update();

                    // Spline move from (0, 0) to (50, 10)
                    drive.actionBuilder(initialPose).splineTo(new Vector2d(50, 10), Math.toRadians(90)).waitSeconds(2).build();
                    currentState = State.STRAFE_TO_POSITION; // Transition to the next state
                    break;


            }

            // Update telemetry to track the state and any relevant data
            telemetry.addData("Current State", currentState);
            telemetry.update();
        }
    }
}
