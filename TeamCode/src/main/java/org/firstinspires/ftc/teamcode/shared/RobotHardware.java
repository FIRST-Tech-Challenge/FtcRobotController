package org.firstinspires.ftc.teamcode.shared;


import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

public class RobotHardware {

    private LinearOpMode myOpMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo leftGripper = null;
    private Servo rightGripper = null;
    private Servo wrist = null;
    private DcMotor armMotor = null;

    // Variables

    public static final double FORWARD_SPEED = 0.4;
    public static final double TURN_SPEED = 0.4;
    public static double detectWait = 6.0;
    public static double wristStart = 0.4;
    public static double wristRight = 0;
    public static double wristLeft = 1;

    public RobotHardware (LinearOpMode opmode) { myOpMode = opmode; }

    public void init()   {

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftGripper = myOpMode.hardwareMap.get(Servo.class, "leftGripper");
        rightGripper = myOpMode.hardwareMap.get(Servo.class, "rightGripper");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "armMotor");
        wrist = myOpMode.hardwareMap.servo.get("wristServo");
        wrist.setPosition(.4);
        leftGripper.setPosition(1); // Adjust the position value as needed
        rightGripper.setPosition(0); // Adjust the position value as needed

        myOpMode.sleep(1000);

        leftGripper.setPosition(1); // Adjust the position value as needed
        rightGripper.setPosition(0); // Adjust the position value as needed

    }

    public void moveRobot(String path, double time) {

        switch(path.toLowerCase(Locale.ROOT)) {
            case "back":
                frontLeftMotor.setPower(FORWARD_SPEED);
                backLeftMotor.setPower(FORWARD_SPEED);
                frontRightMotor.setPower(FORWARD_SPEED);
                backRightMotor.setPower(FORWARD_SPEED);
                runtime.reset();
                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                    telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            default:
                frontLeftMotor.setPower(-FORWARD_SPEED);
                backLeftMotor.setPower(-FORWARD_SPEED);
                frontRightMotor.setPower(-FORWARD_SPEED);
                backRightMotor.setPower(-FORWARD_SPEED);
                runtime.reset();
                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                    telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
        }
    }

}

