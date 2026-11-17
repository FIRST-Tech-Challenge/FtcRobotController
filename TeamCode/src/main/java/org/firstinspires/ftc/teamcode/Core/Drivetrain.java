package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    private NewHardwareMap newHardwareMap;

    private final DcMotorWrapper frontRightMotor;
    private final DcMotorWrapper frontLeftMotor;
    private final DcMotorWrapper backLeftMotor;
    private final DcMotorWrapper backRightMotor;
    Gamepad gamepad;
    Telemetry telemetry;


    public Drivetrain(Telemetry telemetry, Gamepad gamepad, MechanumHardwareMap mechanumHardwareMap) {
        frontRightMotor = new DcMotorWrapper(mechanumHardwareMap.getFrontRightMotor(), 1);
        frontLeftMotor = new DcMotorWrapper(mechanumHardwareMap.getFrontLeftMotor(), 1);
        backRightMotor = new DcMotorWrapper(mechanumHardwareMap.getBackRightMotor(), 1);
        backLeftMotor = new DcMotorWrapper(mechanumHardwareMap.getBackLeftMotor(), 1);
        this.telemetry = telemetry;
        this.gamepad = gamepad;

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        double verticalMovement = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x * 1.1;
        double turning = gamepad.right_stick_x;
        double maintainRatio = Math.max(Math.abs(verticalMovement) + Math.abs(strafe) + Math.abs(turning), 1);

        frontRightMotor.setPower((verticalMovement - strafe - turning) / maintainRatio);
        backRightMotor.setPower((verticalMovement + strafe - turning) / maintainRatio);
        frontLeftMotor.setPower((verticalMovement + strafe + turning) / maintainRatio);
        backLeftMotor.setPower((verticalMovement - strafe + turning) / maintainRatio);

        telemetry.addData("FRM Power", frontRightMotor.getPower());
        telemetry.addData("FLM Power", frontLeftMotor.getPower());
        telemetry.addData("BRM Power", backRightMotor.getPower());
        telemetry.addData("BLM Power", backLeftMotor.getPower());

    }
}
