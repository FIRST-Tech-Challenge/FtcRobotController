package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Common.MyRobot;

public class DriveSubsystem extends SubsystemBase {

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    public DriveSubsystem(MyRobot robot) {
        leftMotor = robot.hardwareMap().get(DcMotorEx.class, "leftMotor");
        rightMotor = robot.hardwareMap().get(DcMotorEx.class, "rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void arcadeDrive(double forward, double turn) {
        double leftPower = forward + turn;
        double rightPower = forward - turn;
        double scale = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));

        leftMotor.setPower(Range.clip(leftPower / scale, -1.0, 1.0));
        rightMotor.setPower(Range.clip(rightPower / scale, -1.0, 1.0));
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
