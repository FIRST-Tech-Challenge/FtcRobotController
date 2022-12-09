package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {
    public static DcMotor leftLift;
    public static DcMotor rightLift;
    public static CRServo gripper;

    public Arm(){}

    public static void init(DcMotor leftLift, DcMotor rightLift, CRServo gripper){
        Arm.leftLift = leftLift;
        Arm.rightLift = rightLift;
        Arm.gripper = gripper;


        leftLift.setDirection(DcMotor.Direction.REVERSE);
    }
}
