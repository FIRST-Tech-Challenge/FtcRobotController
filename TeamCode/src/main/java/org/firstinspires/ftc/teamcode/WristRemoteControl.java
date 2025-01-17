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

@TeleOp(name="Wrist Remote Control", group="Linear OpMode")
public class WristRemoteControl extends LinearOpMode {
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
    boolean wheelClimb = false;

    // Collect joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // This chunk controls our vertical
    DcMotor vertical = null;
    final int VERTICAL_MIN = 0;
    final int VERTICAL_MAX = 2155; // todo: fine-tune this value
    final int VERTICAL_MAX_VIPER = 1200;
    final int VERTICAL_CLIMB_POSITION = 2300;
    final int VERTICAL_DRIVE_POSITION = 400;
    final int VERTICAL_DEFAULT_SPEED = 2000;
    int verticalAdjustedMin = 0;
    int verticalPosition = VERTICAL_MIN;

    // This chunk controls our viper slide
    DcMotor viperSlide = null;
    final int VIPER_MAX_WIDE = 2000;
    final int VIPER_MAX_TALL = 2637;
    final int VIPER_MIN = 0;
    int viperSlidePosition = 0;

    // This chunk controls our claw
    Servo claw = null;
    final double CLAW_MIN = 0.2;        // Claw is closed
    final double CLAW_MAX = 0.6;       // Claw is open - Og non-wrist value was 0.8
    double claw_position = CLAW_MAX;

    // This chunk controls our wrist (todo: update values)
    Servo wrist = null;
    final double WRIST_PICKUP = 0.2;       // Wrist is in intake position (picking up)
    final double WRIST_DROPOFF = 0.8;      // Wrist is in outtake position (dropping in basket)
    double wrist_position = WRIST_DROPOFF;

    // This chunk controls our nose picker (ascent stick)
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
        telemetry.addData("This code was last updated", "1/17/2024, 11:32 am"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        setAscentStick(ASCENT_MIN);
        //claw.setPosition(CLAW_MAX);
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // Get input from the joysticks
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            setWheelPower();

            if(!wheelClimb) {
                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            // Control the vertical - the rotation level of the arm
            verticalPosition = vertical.getCurrentPosition();
            verticalAdjustedMin = (int)(0.07*viperSlidePosition+VERTICAL_MIN); // 0.07 - If the viper is hits the ground, make this bigger. If it doesn't down far enough, make this smaller.

            // Set vertical into initial climb position
            if (gamepad1.dpad_up) {
                // Hook onto the bar
                setVertical(VERTICAL_CLIMB_POSITION, 2500);
            }
            // Active climb
            else if (gamepad1.dpad_down) {
                if (vertical.getCurrentPosition() > 100) {
                    wheelClimb = true;
                    vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    vertical.setPower(-0.8);
                    leftBackDrive.setPower(0.5);
                    rightBackDrive.setPower(0.5);
                }
                else {
                        leftBackDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        wheelClimb = false;
                        setVertical(VERTICAL_MIN, 1000);
                }
            }

            // If the right button is pressed AND it can safely raise further
            else if (gamepad1.dpad_right && verticalPosition < VERTICAL_MAX) {
                setVertical(Math.min(VERTICAL_MAX, verticalPosition + 50), 2000);
            }
            // If the left button is pressed AND it can safely lower without changing the viper
            else if (gamepad1.dpad_left && verticalPosition > VERTICAL_MAX_VIPER) {
                setVertical(Math.max(VERTICAL_MAX_VIPER, verticalPosition - 50), 1500);
            }
            // If the left button is pressed AND it can safely lower further
            else if (gamepad1.dpad_left && verticalPosition > verticalAdjustedMin) {
                if (viperSlidePosition > VIPER_MAX_WIDE) {
                    setViper(VIPER_MAX_WIDE, 1000);
                }
                setVertical(Math.max(verticalAdjustedMin, verticalPosition - 50),1000);
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
            if (gamepad1.right_bumper && claw.getPosition() < CLAW_MAX) {
                setClaw(claw.getPosition() + 0.3);
            }
            if (gamepad1.left_bumper && claw.getPosition() > CLAW_MIN) {
                setClaw(claw.getPosition() - 0.15);
            }

            // Y/Triangle: High basket scoring position.
            if (gamepad1.y) {
                tScoringPosition();
            }

            // A/X button: Complete Retraction- Viper and vertical completely retracted and down
            if (gamepad1.a) {
                xClosedPosition();
            }

            // X/Square: The viper slide is completely retracted but the vertical is in submersible position.
            if (gamepad1.x) {
                sDrivingPosition();
            }

            // B/Circle: The vertical is in submersible position and the viper slide is all the way out.
            if (gamepad1.b) {
                oPickupPosition();
            }

            // Show the elapsed game time and wheel power.
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
        claw.setPosition(CLAW_MAX);

        // todo: check this
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        setWrist(WRIST_DROPOFF);

        ascentStick = hardwareMap.get(Servo.class, "ascentStick");
        ascentStick.setDirection(Servo.Direction.REVERSE);
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

    public void tScoringPosition() {
        RobotLog.vv("Rockin' Robots", "Get in scoring position");
        while (vertical.getCurrentPosition() < 1000 && viperSlide.getCurrentPosition() > 500) {
            RobotLog.vv("Rockin' Robots", "6");
            setViper(VIPER_MIN, 4000);
        }
        while (vertical.getCurrentPosition() < 1700) {
            RobotLog.vv("Rockin' Robots", "7");
            setVertical(VERTICAL_MAX, 3000);
        }
        setViper(VIPER_MAX_TALL, 4000);
        setWrist(WRIST_DROPOFF);
    }

    public void oPickupPosition() {
        RobotLog.vv("Rockin' Robots", "Get in pickup position");
        while (viperSlide.getCurrentPosition() > 500) {
            RobotLog.vv("Rockin' Robots", "1");
            setViper(VIPER_MIN, 4000);
        }
        setVertical(200, 3000);
        while (vertical.getCurrentPosition() > 500) {
            RobotLog.vv("Rockin' Robots", "2");
            setVertical(200, 3000);
        }
        setViper(VIPER_MAX_WIDE, 4000);
        setWrist(WRIST_PICKUP);
        setClaw(CLAW_MAX);
    }
//MILLA FIX THIS the order we want of the viper and vertical moving changes based on the current robot position
    public void sDrivingPosition() {
        RobotLog.vv("Rockin' Robots", "sDrivingPosition()");
        if (vertical.getCurrentPosition() < 500)
        {
            RobotLog.vv("Rockin' Robots", "sDrivingPosition(): Starting from low position");
            setVertical(VERTICAL_DRIVE_POSITION, 3000);
            setViper(VIPER_MIN, 4000);
            while (viperSlide.getCurrentPosition() > 500) {
                RobotLog.vv("Rockin' Robots", "sDrivingPosition(): while waiting for viper from low position");
                setViper(VIPER_MIN, 4000);
            }
        }
        else {
            RobotLog.vv("Rockin' Robots", "sDrivingPosition(): Starting from high position");
            setViper(VIPER_MIN, 4000);
            while (viperSlide.getCurrentPosition() > 500) {
                RobotLog.vv("Rockin' Robots", "sDrivingPosition(): while waiting for viper from high position");
                setViper(VIPER_MIN, 4000);
            }
            setVertical(VERTICAL_DRIVE_POSITION, 3000);
        }
        setWrist(WRIST_PICKUP);
    }

    public void xClosedPosition() {
        RobotLog.vv("Rockin' Robots", "Get in closed position");
        setWrist(WRIST_DROPOFF);
        setViper(VIPER_MIN, 4000);
        while (viperSlide.getCurrentPosition() > 900) {
            RobotLog.vv("Rockin' Robots", "4");
            setViper(VIPER_MIN, 4000);
        }
        setVertical(VERTICAL_MIN, 3000);
        while (vertical.getCurrentPosition() > 500) {
            RobotLog.vv("Rockin' Robots", "72");
            setVertical(VERTICAL_MIN, 3000);
        }
        setClaw(CLAW_MIN);
    }

    public void setAscentStick(double target) {
        RobotLog.vv("Rockin' Robots", "Set Ascent Stick to: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
        ascentStick.setPosition(target);
        //sleep(1000);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, ascentStick.getPosition());
    }

    public void setClaw(double target) {
        RobotLog.vv("Rockin' Robots", "Set Claw to: %4.2f, Current: %4.2f", target, claw.getPosition());
        claw.setPosition(target);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, claw.getPosition());
    }

    public void setWrist(double target) { // todo: check this method
        RobotLog.vv("Rockin' Robots", "Set Wrist to: %4.2f, Current: %4.2f", target, wrist.getPosition());
        wrist.setPosition(target);
        //sleep(1000);
        RobotLog.vv("Rockin' Robots", "Target: %4.2f, Current: %4.2f", target, wrist.getPosition());
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
        telemetry.addData("wrist position", "%4.2f", wrist.getPosition());
        //RobotLog.vv("Rockin", "Vert Velocity: %.1f, Vert Power: %.1f, Vert Power Consumption: %.1f, Vert Position: %d",
        //        ((DcMotorEx) vertical).getVelocity(),  vertical.getPower(), ((DcMotorEx)vertical).getCurrent(CurrentUnit.AMPS), vertical.getCurrentPosition());

        telemetry.update();
    }
}