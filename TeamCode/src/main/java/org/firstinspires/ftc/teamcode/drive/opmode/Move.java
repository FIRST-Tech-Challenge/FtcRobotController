package org.firstinspires.ftc.teamcode.drive.opmode;
//package org.openftc.i2cdrivers;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;



@TeleOp
@I2cDeviceType()

public class Move extends OpMode {

    Gamepad.RumbleEffect Endgame;
    Gamepad.RumbleEffect Park;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    public float time_elapsed = 0;
    public boolean Endgametrue = false;
    public boolean Parktrue = false;
    final double Endgamefloat = 80.0;
    final double EndOfGamefloat = 115.0;
    private static final boolean ON = true;
    private static final boolean OFF = false;

    ElapsedTime runtime = new ElapsedTime();

    static void sleep(int LongMilliseconds) {
        try {
            Thread.sleep(LongMilliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void init() {
        // Initialize gamepad rumble effects

        telemetry.addLine(">>>>>> PRESS START BUTTON");
        telemetry.update();

        // Initialize DcMotors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        time_elapsed = 0;
        telemetry.clearAll();
        Endgametrue = false;
        Parktrue = false;
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower; // Declare motor power variables

        // Set drive controls
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Set motor power
        leftFrontPower = Range.clip(drive + turn + strafe, -0.35, 0.35);
        rightFrontPower = Range.clip(drive - turn - strafe, -0.35, 0.35);
        leftBackPower = Range.clip(drive + turn - strafe, -0.35, 0.35);
        rightBackPower = Range.clip(drive - turn + strafe, -0.35, 0.35);

        // Telemetry
        telemetry.addData("Speed: ", (leftFrontPower + leftBackPower + rightBackPower + rightFrontPower) / 4);
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y: ", -gamepad1.left_stick_y);
        telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y: ", -gamepad1.right_stick_y);
        telemetry.update();

        // Endgame timer
        if (gamepad1.y || gamepad1.x) {
            leftFrontPower = -0.16;
            leftBackPower = -0.16;
            rightFrontPower = -0.16;
            rightBackPower = -0.16;
        }

        // Slow movement
        if (gamepad1.right_bumper) {
            leftFrontPower /= 4;
            leftBackPower /= 4;
            rightFrontPower /= 4;
            rightBackPower /= 4;
        }
        else if(gamepad1.left_bumper){
            leftFrontPower *= 1.25;
            leftBackPower *= 1.25;
            rightFrontPower *= 1.25;
            rightBackPower *= 1.25;
        }



        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void stop() {
        // Stop all motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}

