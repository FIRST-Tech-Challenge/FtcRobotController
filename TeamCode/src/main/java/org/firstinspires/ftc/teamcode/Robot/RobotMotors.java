package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMotors {
    public DcMotorEx frontLeftDrive;
    public DcMotorEx frontRightDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;

    public RobotMotors(HardwareMap hardwareMap){
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotorEx. class, "back_right)drive");
    }
}
