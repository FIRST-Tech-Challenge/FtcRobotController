package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** @noinspection unused*/
public class Hardware {

    // Consts
    public Double lateralEncoderDist = 12.96;
    public Double longEncoderDist = 238.125;

    // Motors
    public static DcMotorEx motorTL;
    public static DcMotorEx motorTR;
    public static DcMotorEx motorBL;
    public static DcMotorEx motorBR;

    // Encoders
    public static OverflowEncoder encoderBottom;
    public static OverflowEncoder encoderLeft;
    public static OverflowEncoder encoderRight;

    HardwareMap hardwareMap;


    public Hardware(HardwareMap map) {
        this.hardwareMap = map;

        hardwareMap.get(DcMotorEx.class, "topLeft");

        motorTL = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorTR = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBL = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBR = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderBottom = new OverflowEncoder(new RawEncoder(motorTL));
        encoderLeft = new OverflowEncoder(new RawEncoder(motorTR));
        encoderRight = new OverflowEncoder(new RawEncoder(motorBL));

    }
}
