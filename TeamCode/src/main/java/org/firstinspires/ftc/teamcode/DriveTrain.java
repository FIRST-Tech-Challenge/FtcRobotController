package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This controls the drive train
 * @author Joshua Miller <22jmiller@xbhs.net>
 * @author Brayden Meech <22bmeech@xbhs.net>
 * @version 1.1.0
 */

public class DriveTrain {
    // Default name
    private static final String DEFAULT_LEFT_MOTOR_NAME = "left_drive";
    private static final String DEFAULT_RIGHT_MOTOR_NAME = "right_drive";

    // Drive Modes
    public static final int DRIVE_MODE_WHEEL_PIVOT = 0;
    public static final int DRIVE_MODE_STOP = -1;

    // Debugging
    public boolean debugging;

    public static final int DRIVE_MODE_DEFAULT = DRIVE_MODE_WHEEL_PIVOT;

    // Gamepad
    Gamepad gamepad1;

    // Motors
    DcMotor leftMotor;
    DcMotor rightMotor;

    // Drive mode
    private int driveMode;

    public DriveTrain(Gamepad gamepad1, DcMotor leftMotor, DcMotor rightMotor, int driveMode) {
        this.gamepad1 = gamepad1;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.driveMode = driveMode;

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        debugging = false;
    }

    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1, String leftMotorName, String rightMotorName, int driveMode) {
        this(gamepad1, hardwareMap.get(DcMotor.class, leftMotorName), hardwareMap.get(DcMotor.class, rightMotorName), driveMode);
    }

    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1, int driveMode) {
        this(hardwareMap, gamepad1, DEFAULT_LEFT_MOTOR_NAME, DEFAULT_RIGHT_MOTOR_NAME, driveMode);
    }

    public void setDebugging(boolean debugging) {
        this.debugging = debugging;
    }

    public void update() {
        switch (driveMode) {
            case DRIVE_MODE_WHEEL_PIVOT:
                double rightPower = 2 * gamepad1.left_stick_y * ((Math.abs(gamepad1.left_stick_x - 1) / 2));
                double leftPower = 2 * gamepad1.left_stick_y * ((Math.abs(gamepad1.left_stick_x + 1) / 2));

                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
                break;
            case DRIVE_MODE_STOP:
            default:
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
        }
    }
}
