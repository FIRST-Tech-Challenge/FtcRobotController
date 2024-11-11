package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    // Initialize all variables for the program below:
    // This chunk controls our wheels
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;
    boolean wheelClimb = false;

    // Collect joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 1700;
    final int VERTICAL_MAX_VIPER = 1200;
    int verticalAdjustedMin = 0;
    int verticalPosition = VERTICAL_MIN;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX_WIDE = 2500;
    final int VIPER_MAX_TALL = 3100;
    final int VIPER_MIN = 0;
    int viperSlidePosition = 0;

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.05;        // Claw is closed
    final double CLAW_MAX = 0.43;        // Claw is open
    double claw_position = CLAW_MIN;

    final ElapsedTime runtime = new ElapsedTime();

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {
        // Initialize the hardware variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(CLAW_MIN);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");     // TODO: create a better message
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Get input from the joysticks
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power.
            leftFrontPower = (axial + lateral + yaw) / 2;
            rightFrontPower = (axial - lateral - yaw) / 2;
            leftBackPower = (axial - lateral + yaw) / 2;
            rightBackPower = (axial + lateral - yaw) / 2;

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if(wheelClimb == false) {
                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            // Control the vertical - the rotation level of the arm
            verticalPosition = vertical.getCurrentPosition();
            verticalAdjustedMin = (int)(0.07*viperSlidePosition+VERTICAL_MIN); // 0.07 - If the viper is hitting the ground, make this bigger. If it's not going down far enough, make this smaller.

            // Setting vertical into initial climb position
            if (gamepad1.dpad_up) {
                wheelClimb = true;
                // Hook on to the bar
                vertical.setTargetPosition(2800);                       // todo: This is a magic number so it should be a CAPS variable at the top
                                                                        // todo: This likely causes us to exceed the max length. What should we do about that?
                ((DcMotorEx) vertical).setVelocity(3000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.dpad_down) {
                if (vertical.getCurrentPosition() > 100) {
                    wheelClimb = true;
                    vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    vertical.setPower(-0.8);
                    leftBackDrive.setPower(0.5);
                    rightBackDrive.setPower(0.5);
                }
                else {
                    wheelClimb = false;
                    vertical.setTargetPosition(VERTICAL_MIN);
                    ((DcMotorEx) vertical).setVelocity(1000);
                    vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            // If the right button is pressed AND it can safely raise further
            else if (gamepad1.dpad_right && verticalPosition < VERTICAL_MAX) {
                vertical.setTargetPosition(Math.min(VERTICAL_MAX, verticalPosition + 50));
                ((DcMotorEx) vertical).setVelocity(2000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // todo: missing a comment
            else if (gamepad1.dpad_left && verticalPosition > VERTICAL_MAX_VIPER) {
                vertical.setTargetPosition(Math.max(VERTICAL_MAX_VIPER, verticalPosition - 50));
                ((DcMotorEx) vertical).setVelocity(1500);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // If the left button is pressed AND it can safely lower further
            else if (gamepad1.dpad_left && verticalPosition > verticalAdjustedMin) {
                if (viperSlidePosition > VIPER_MAX_WIDE) {
                    viperSlide.setTargetPosition(VIPER_MAX_WIDE);
                    ((DcMotorEx) viperSlide).setVelocity(1000);
                    viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                vertical.setTargetPosition(Math.max(verticalAdjustedMin, verticalPosition - 50));
                ((DcMotorEx) vertical).setVelocity(1000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Control the viper slide - how much it extends
            viperSlidePosition = viperSlide.getCurrentPosition();
            // If the right button is pressed AND it can safely extend further, and the viper can go all the way up
            if (gamepad1.right_trigger > 0 && viperSlidePosition < VIPER_MAX_TALL && verticalPosition > VERTICAL_MAX_VIPER) {
                viperSlide.setTargetPosition(viperSlidePosition + 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.right_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // If the right button is pressed AND it can safely extend further, and the viper is under the wide limit
            else if (gamepad1.right_trigger > 0 && viperSlidePosition < VIPER_MAX_WIDE && verticalPosition < VERTICAL_MAX_VIPER) {
                viperSlide.setTargetPosition(viperSlidePosition + 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.right_trigger * 4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // If the right button is pressed AND it can safely retract further
            else if (gamepad1.left_trigger > 0 && viperSlidePosition > VIPER_MIN) {
                viperSlide.setTargetPosition(Math.max(VIPER_MIN, viperSlidePosition - 200));
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.left_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Control the claw
            if (gamepad1.right_bumper && claw_position > CLAW_MAX) {
                claw_position -= 0.02;
            }
            if (gamepad1.left_bumper && claw_position < CLAW_MIN) {
                claw_position += 0.015;
            }
            claw.setPosition(claw_position);

            // Y/Triangle: High basket scoring position.
            if (gamepad1.y) {
                vertical.setTargetPosition(VERTICAL_MAX);
                ((DcMotorEx) vertical).setVelocity(3000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlide.setTargetPosition(VIPER_MAX_TALL);
                ((DcMotorEx) viperSlide).setVelocity(2000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // A/X button: Bring the viper slide and put the vertical all the way down. // todo this comment isn't correct
            if (gamepad1.a) {
                viperSlide.setTargetPosition(VIPER_MIN);
                ((DcMotorEx) viperSlide).setVelocity(4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical.setTargetPosition(VERTICAL_MIN);
                ((DcMotorEx) vertical).setVelocity(700);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // X/Square: The viper slide is completely back but the vertical is in submersible position. // todo: instead of back, you should use retracted
            if (gamepad1.x) {
                vertical.setTargetPosition(350);                        // todo: does this value work on both sides of the submersible?
                ((DcMotorEx) vertical).setVelocity(3000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlide.setTargetPosition(0);
                ((DcMotorEx) viperSlide).setVelocity(1500);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // B/Circle: The vertical is in submersible position and the viper slide is all the way out.
            if (gamepad1.b) {
                vertical.setTargetPosition(350);                         // todo: does this value work on both sides of the submersible?
                ((DcMotorEx) vertical).setVelocity(1800);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlide.setTargetPosition(1900);
                ((DcMotorEx) viperSlide).setVelocity(2000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Show the elapsed game time and wheel power.
            printDataOnScreen();
        }
        claw.close();
        // todo: add some code to protect our robot after the hang
    }

    // Log all (relevant) info about the robot on the hub.
    private void printDataOnScreen() {
        telemetry.addData("Run Time", "%.1f", runtime.seconds());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //RobotLog.vv("RockinRobots", "%4.2f, %4.2f, %4.2f, %4.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Joystick Axial", "%4.2f", axial);
        telemetry.addData("Joystick Lateral", "%4.2f", lateral);
        telemetry.addData("Joystick Yaw", "%4.2f", yaw);
        telemetry.addData("Claw position", "%4.2f", claw_position); // todo: we should print both the targeted position (this) and the actual position (claw.getPosition())
        telemetry.addData("Viper Slide Velocity", "%4.2f", ((DcMotorEx) viperSlide).getVelocity());
        telemetry.addData("Viper power consumption", "%.1f", ((DcMotorEx) viperSlide).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Viper Slide Position", "%d", viperSlidePosition);
        telemetry.addData("Vertical Power", "%.1f", ((DcMotorEx) vertical).getVelocity());
        telemetry.addData("Vertical power consumption", "%.1f", ((DcMotorEx) vertical).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Position", "%d", vertical.getCurrentPosition());
        telemetry.addData("Vertical Adjusted Min", "%d", verticalAdjustedMin);
        RobotLog.vv("Rockin", "Vert Velocity: %.1f, Vert Power: %.1f, Vert Power Consumption: %.1f, Vert Position: %d",
                ((DcMotorEx) vertical).getVelocity(),  vertical.getPower(), ((DcMotorEx)vertical).getCurrent(CurrentUnit.AMPS), vertical.getCurrentPosition());

        telemetry.update();
    }
}