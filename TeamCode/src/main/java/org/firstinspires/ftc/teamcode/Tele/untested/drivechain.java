package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.hardware.DcMotor;

public class drivechain {
    static DcMotor motorFrontLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorBackRight = null;

    public static void setDTMotors(DcMotor FL,DcMotor FR,DcMotor BL,DcMotor BR){
        motorFrontLeft=FL;
        motorFrontRight=FR;
        motorBackLeft = BL;
        motorBackRight = BR;
    }


}