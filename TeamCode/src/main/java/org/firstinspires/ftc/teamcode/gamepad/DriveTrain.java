package org.firstinspires.ftc.teamcode.gamepad;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.settings.HardwareMapDeviceNames;

/**
 * This controls the drive train
 * @author Joshua Miller <22jmiller@xbhs.net>
 * @author Brayden Meech <22bmeech@xbhs.net>
 * @version 1.2.0
 */

public class DriveTrain {
    // Drive Modes
    public static final int DRIVE_MODE_STOP = -1;
    public static final int DRIVE_MODE_WHEEL_PIVOT = 0;
    public static final int DRIVE_MODE_TANK = 1;
    public static final int DRIVE_MODE_MIDDLE_PIVOT = 2;

    // Debugging
    private boolean debugging;

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
        this(hardwareMap, gamepad1, HardwareMapDeviceNames.LEFT_DRIVE, HardwareMapDeviceNames.RIGHT_DRIVE, driveMode);
    }

    public void setDebugging(boolean debugging) {
        this.debugging = debugging;
    }

    public void update() {
        switch (driveMode) {
            // TODO: This doesn't work as well as expected
            case DRIVE_MODE_WHEEL_PIVOT:
                double rightPower = 2 * gamepad1.left_stick_y * ((Math.abs(gamepad1.left_stick_x - 1) / 2));
                double leftPower = 2 * gamepad1.left_stick_y * ((Math.abs(gamepad1.left_stick_x + 1) / 2));

                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower);
                break;
            case DRIVE_MODE_TANK:
                leftMotor.setPower(gamepad1.left_stick_y);
                rightMotor.setPower(gamepad1.right_stick_y);
                break;
            case DRIVE_MODE_MIDDLE_PIVOT:
                // Distance from center
                double power = Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2));

                if (gamepad1.left_stick_y <= 0) { // Going Forwards
                    leftMotor.setPower(2*gamepad1.left_stick_x+power);
                    rightMotor.setPower(-2*gamepad1.left_stick_x+power);
                } else { // Backwards
                    leftMotor.setPower(2*gamepad1.left_stick_x-power);
                    rightMotor.setPower(-2*gamepad1.left_stick_x-power);
                }

                break;
            case DRIVE_MODE_STOP:
            default:
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
        }
    }
}
