package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis {
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
}
