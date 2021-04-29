package org.firstinspires.ftc.teamcode.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.settings.DeviceNames;

import java.util.ArrayList;

public class DriveTrain {
    DcMotor[] leftMotors;
    DcMotor[] rightMotors;

    public DriveTrain(DcMotor[] leftMotors, DcMotor[] rightMotors) {
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
    }

    public DriveTrain(HardwareMap hardwareMap, String[] leftMotorNames, String[] rightMotorNames) {
        // Left Drive
        ArrayList<DcMotor> leftMotorsArrayList = new ArrayList<DcMotor>();
        for (String name : DeviceNames.LEFT_DRIVE) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotorsArrayList.add(motor);
        }
        leftMotors = (DcMotor[]) leftMotorsArrayList.toArray();

        // Right Drive
        ArrayList<DcMotor> rightMotorsArrayList = new ArrayList<DcMotor>();
        for (String name : DeviceNames.RIGHT_DRIVE) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            rightMotorsArrayList.add(motor);
        }
        rightMotors = (DcMotor[]) rightMotorsArrayList.toArray();
    }

    public DriveTrain(Robot robot) {
        this(robot.hardwareMap, robot.deviceNames.LEFT_DRIVE, robot.deviceNames.RIGHT_DRIVE);
    }
}
