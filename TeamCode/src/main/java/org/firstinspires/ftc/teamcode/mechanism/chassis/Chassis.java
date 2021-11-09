package org.firstinspires.ftc.teamcode.mechanism.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Mechanism;

public abstract class Chassis implements Mechanism {
    public static DcMotor frontRight;
    public static DcMotor frontLeft;
    public static DcMotor backRight;
    public static DcMotor backLeft;
    public void init(HardwareMap hardwareMap){
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
    }
    public abstract void run(Gamepad gamepad);


}
