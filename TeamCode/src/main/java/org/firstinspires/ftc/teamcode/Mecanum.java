package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mecanum {
    // Motors
    private DcMotor leftFrontDrive   = null;
    private DcMotor  rightFrontDrive  = null;
    private DcMotor  leftRearDrive    = null;
    private DcMotor  rightRearDrive   = null;

    // Variables
    private double driveSpeed = 1.0;

    // Debug
    private boolean debug = true;
    private Telemetry telemetry;

    public Mecanum(HardwareMap hardwareMap) {
        //Drive base
        leftFrontDrive  = hardwareMap.dcMotor.get("leftFrontDrive");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive");
        leftRearDrive = hardwareMap.dcMotor.get("leftRearDrive");
        rightRearDrive = hardwareMap.dcMotor.get("rightRearDrive");

        //this needs to be corrected with testing, this is just and example
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setDriveSpeed(double driveSpeed) {
        this.driveSpeed = driveSpeed;
    }

    public void setDebug(boolean toggle) {
        this.debug = toggle;
    }

    public void update(Gamepad gamepad) {
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        double leftFrontWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) + rightX;
        double rightFrontWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) - rightX;
        double leftRearWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) + rightX;
        double rightRearWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) - rightX;

        leftFrontDrive.setPower(leftFrontWheelPower * driveSpeed);
        rightFrontDrive.setPower(rightFrontWheelPower * driveSpeed);
        leftRearDrive.setPower(leftRearWheelPower * driveSpeed);
        rightRearDrive.setPower(rightRearWheelPower * driveSpeed);

        if (this.debug) {
            telemetry.addData("leftFront",  "%.2f", leftFrontWheelPower);
            telemetry.addData("rightFront",  "%.2f", rightFrontWheelPower);
            telemetry.addData("leftRear",  "%.2f", leftRearWheelPower);
            telemetry.addData("rightRear", "%.2f", rightRearWheelPower);
        }
    }
}
