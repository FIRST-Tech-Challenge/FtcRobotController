package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="SurvivorCode", group="Linear OpMode")
@Disabled
public class SurvivorCode extends LinearOpMode {
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
    double max = 0;

    // Collect joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 1700;
    final int VERTICAL_MAX_VIPER = 1200;
    final int VERTICAL_DEFAULT_SPEED = 2000;
    int verticalAdjustedMin = 0;
    int verticalPosition = VERTICAL_MIN;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX_WIDE = 2100;
    final int VIPER_MAX_TALL = 3100;
    final int VIPER_MIN = 0;
    int viperSlidePosition = 0;

    // This chunk controls our claw
    //Callie
    Servo claw = null;
    final double CLAW_MIN = 0.2;        // Claw is closed
    final double CLAW_MAX = 0.8;        // Claw is open
    double claw_position = CLAW_MIN;

    Servo ascentStick = null;
    final double ASCENT_MIN = 0.2;          // Stick is down
    final double ASCENT_MAX = 0.49;         // Stick is up

    final ElapsedTime runtime = new ElapsedTime();

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        initializeHardwareVariables();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        telemetry.update();
        setAscentStick(ASCENT_MIN);
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // Get input from the joysticks
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            setWheelPower();
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Control the vertical - the rotation level of the arm
            verticalPosition = vertical.getCurrentPosition();
            verticalAdjustedMin = (int)(0.07*viperSlidePosition+VERTICAL_MIN); // 0.07 - If the viper is hits the ground, make this bigger. If it doesn't down far enough, make this smaller.

            // If the right button is pressed AND it can safely raise further
            if (gamepad1.dpad_right) {
                setVertical(Math.min(VERTICAL_MAX, verticalPosition + 50), 2000);
            }
            // If the left button is pressed AND it can safely lower without changing the viper
            else if (gamepad1.dpad_left) {
                setVertical( verticalPosition - 630);
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
            else if (gamepad1.left_trigger > 0) {
                viperSlide.setTargetPosition(viperSlidePosition - 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.left_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Control the claw
            if (gamepad1.right_bumper && claw_position < CLAW_MAX) {
                claw_position += 0.02;
            }
            if (gamepad1.left_bumper && claw_position > CLAW_MIN) {
                claw_position -= 0.015;
            }
            claw.setPosition(claw_position);

            printDataOnScreen();
        }
        claw.close();
    }

    private void initializeHardwareVariables() {
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

        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
        ascentStick.setPosition(ASCENT_MIN);
    }
    private void setWheelPower(){
        leftFrontPower = (axial + lateral + yaw) / 2;
        rightFrontPower = (axial - lateral - yaw) / 2;
        leftBackPower = (axial - lateral + yaw) / 2;
        rightBackPower = (axial + lateral - yaw) / 2;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
    }

    public void setAscentStick(double target) {
        RobotLog.vv("Rockin' Robots", "Set Ascent Stick to: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
        ascentStick.setPosition(target);
        sleep(1000);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
    }

    public void setVertical(int height){
        setVertical(height, VERTICAL_DEFAULT_SPEED);
    }

    public void setVertical(int height, int speed){
        vertical.setTargetPosition(height);
        ((DcMotorEx) vertical).setVelocity(speed);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setViper(int length, int speed){
        viperSlide.setTargetPosition(length);
        ((DcMotorEx) viperSlide).setVelocity(speed);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotLog.vv("Rockin' Robots", "Viper set to %d", viperSlide.getCurrentPosition());
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
        telemetry.addData("Target claw position", "%4.2f", claw_position);
        telemetry.addData("Claw position", "%4.2f", claw.getPosition());
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