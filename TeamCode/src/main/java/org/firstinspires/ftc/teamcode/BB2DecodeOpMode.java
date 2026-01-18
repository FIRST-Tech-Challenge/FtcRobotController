package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class BB2DecodeOpMode extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor coreHex;
    private DcMotor leftDrive;
    private Servo servo;
    private DcMotor rightDrive;
    private static final float slowSpeed = 0.3F;
    private static final int autoDistance = 25;
    private static final int nearVelocity = 1300;
    private static final int midVelocity = 1650;
    private static final int farVelocity = 1900;
    private static final double nearAngle = 0.0;
    private static final double midAngle = 0.60;
    private static final double farAngle = 0.63;
    private static final String TELEOP = "TELEOP";
    private static final String AUTO_BLUE_GOAL = "AUTO BLUE GOAL";
    private static final String AUTO_RED_GOAL = " AUTO RED GOAL";
    private static final String AUTO_BLUE_WALL = "AUTO BLUE WALL";
    private static final String AUTO_RED_WALL = "AUTO RED WALL";
    private static final double WHEELS_INCHES_TO_TICKS = (28 * 5 * 3) / (3 * Math.PI);
    private static final double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    private static final double P = 265;    // Proportional gain to correct error based on how far off the velocity is.

    private String operationSelected = TELEOP;
    private ElapsedTime autoLaunchTimer = new ElapsedTime();
    private ElapsedTime autoDriveTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        servo = hardwareMap.get(Servo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensures the servo is active and ready
        servo.setPosition(0);

        // On initilization the Driver Station will prompt for which OpMode should be run - Auto Blue, Auto Red, or TeleOp
        while (opModeInInit()) {
            operationSelected = selectOperation(operationSelected, gamepad1.psWasPressed());
            telemetry.update();
        }

        waitForStart();

        if (operationSelected.equals(AUTO_BLUE_GOAL)) {
            doAutoBlueGoal();
        } else if (operationSelected.equals(AUTO_RED_GOAL)) {
            doAutoRedGoal();
        } else if (operationSelected.equals(AUTO_BLUE_WALL)) {
            doAutoBlueWall();
        }  else if (operationSelected.equals(AUTO_RED_WALL)) {
            doAutoRedWall();
        } else {
            doTeleOp();
        }
    }

    /**
     * If the PS/Home button is pressed, the robot will cycle through the OpMode options following the if/else statement here.
     * The telemetry readout to the Driver Station App will update to reflect which is currently selected for when "play" is pressed.
     */
    private String selectOperation(String state, boolean cycleNext) {
        if (cycleNext) {
            if (state.equals(TELEOP)) {
                state = AUTO_BLUE_GOAL;
            } else if (state.equals(AUTO_BLUE_GOAL)) {
                state = AUTO_RED_GOAL;
            } else if (state.equals(AUTO_RED_GOAL)) {
                state = AUTO_BLUE_WALL;
            } else if (state.equals (AUTO_BLUE_WALL)) {
                state = AUTO_RED_WALL;
            } else if (state.equals (AUTO_RED_WALL)) {
                state = TELEOP;
            } else {
                telemetry.addData("WARNING", "Unknown Operation State Reached - Restart Program");
            }
        }
        telemetry.addLine("Press Home Button to cycle options");
        telemetry.addData("CURRENT SELECTION", state);
        if (state.equals(AUTO_BLUE_GOAL) || state.equals(AUTO_RED_GOAL) || state.equals(AUTO_BLUE_WALL) || state.equals(AUTO_RED_WALL)) {
            telemetry.addLine("Please remember to enable the AUTO timer!");
        }
        telemetry.addLine("Press START to start your program");
        return state;
    }

    /**
     * If TeleOp was selected or defaulted to, the following will be active upon pressing "play".
     */
    private void doTeleOp() {
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
                manualFeederControl();
                telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();
            }
        }
    }

    /**
     * Controls for the drivetrain. The robot uses a split stick stlye arcade drive.
     * Forward and back is on the left stick. Turning is on the right stick.
     */
    private void splitStickArcadeDrive() {
        float power = 1;

        if (gamepad1.left_trigger > 0) {
            power = slowSpeed;
        }

        float X = gamepad1.right_stick_x;
        float Y = -gamepad1.left_stick_y;
        float leftDrivePower = (Y - X) * power;
        float rightDrivePower = (Y + X) * power;
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
    private void manualFeederControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            coreHex.setPower(0.5);
        } else if (gamepad1.triangle) {
            coreHex.setPower(-0.5);
        }
    }

    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    private void setFlywheelVelocity() {
        if (gamepad1.left_bumper) {
            farShot();
        } else if (gamepad1.right_bumper) {
            midShot();
        } else if (gamepad1.square) {
            nearShot();
        } else {
            flywheel.setVelocity(0);
            coreHex.setPower(0);
        }
    }

//Automatic Flywheel controls used in Auto and TeleOp

    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    private void midShot() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(midVelocity);
        servo.setPosition(midAngle);
        if (flywheel.getVelocity() >= midVelocity - 100) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    private void nearShot() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(nearVelocity);
        servo.setPosition(nearAngle);
        if ((flywheel.getVelocity() >= nearVelocity) ) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The servo will spin until the bumper is released.
     */
    private void farShot() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(farVelocity);
        servo.setPosition(farAngle);
        if (flywheel.getVelocity() >= farVelocity - 100) {
            coreHex.setPower(1);
        } else {
            coreHex.setPower(0);
        }
    }

//Autonomous Code
//For autonomous, the robot will launch the pre-loaded 3 balls then back away from the goal, turn, and back up off the launch line.

    /**
     * For autonomous, the robot is using a timer and encoders on the drivetrain to move away from the target.
     * This method contains the math to be used with the inputted distance for the encoders, resets the elapsed timer, and
     * provides a check for it to run so long as the motors are busy and the timer has not run out.
     */
    private void autoDrive(double speed, int leftDistanceInch, int rightDistanceInch, int timeout_ms) {
        autoDriveTimer.reset();
        leftDrive.setTargetPosition((int) (leftDrive.getCurrentPosition() + leftDistanceInch * WHEELS_INCHES_TO_TICKS));
        rightDrive.setTargetPosition((int) (rightDrive.getCurrentPosition() + rightDistanceInch * WHEELS_INCHES_TO_TICKS));
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy()) && autoDriveTimer.milliseconds() < timeout_ms) {
            idle();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Blue Alliance Autonomous
     * The robot will fire the pre-loaded balls until the 30 second timer ends.
     * Then it will back away from the goal and off the launch line.
     */
    private void doAutoBlueGoal() {
        if (opModeIsActive()) {
            telemetry.addData("RUNNING OPMODE", operationSelected);
            telemetry.update();
            // Fire artifacts
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 10000) {
                nearShot();
                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
                telemetry.update();
            }
            flywheel.setVelocity(0);
            coreHex.setPower(0);

            // Back Up
            autoDrive(1, -autoDistance, -autoDistance, 5000);

            // Turn
            autoDrive(1, -10, 10, 5000);

            // Drive off Line
            autoDrive(1, -45, -45, 5000);
        }
    }

    /**
     * Red Alliance Autonomous
     * The robot will fire the pre-loaded balls until the 30 second timer ends.
     * Then it will back away from the goal and off the launch line.
     */
    private void doAutoRedGoal() {
        if (opModeIsActive()) {
            telemetry.addData("RUNNING OPMODE", operationSelected);
            telemetry.update();
            // Fire artifacts
            autoLaunchTimer.reset();
            while (opModeIsActive() && autoLaunchTimer.milliseconds() < 13000) {
                nearShot();
                telemetry.addData("Launcher Countdown", autoLaunchTimer.seconds());
                telemetry.update();
            }
            flywheel.setVelocity(0);
            coreHex.setPower(0);

            // Back Up
            autoDrive(1, -autoDistance, -autoDistance, 5000);

            // Turn
            autoDrive(1, 10, -10, 5000);

            // Drive off Line
            autoDrive(1, -45, -45, 5000);
        }
    }
    private void doAutoBlueWall() {
        autoDrive(1, 15, 15, 5000);
    }
    private void doAutoRedWall() {
        autoDrive(1, 15, 15, 5000);
    }
}