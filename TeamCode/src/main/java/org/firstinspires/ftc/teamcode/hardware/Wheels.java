package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wheels {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor[] wheelArr= {frontLeft, frontRight, backLeft, backRight};


    public Wheels(HardwareMap hm) {
        frontLeft = hm.get(DcMotor.class, "FrontLeft");
        frontRight = hm.get(DcMotor.class, "FrontRight");
        backLeft = hm.get(DcMotor.class, "BackLeft");
        backRight = hm.get(DcMotor.class, "BackRight");
    }

    public DcMotor toDcMotor(Wheel w) {
        switch (w) {
            case backLeft:
                return backLeft;
            case backRight:
                return backRight;
            case frontLeft:
                return frontLeft;
            case frontRight:
                return frontRight;
        }
        return null;
    }

    public void setPower(Wheel m, double power) {
        DcMotor mot = toDcMotor(m);
        if (mot==null) return;

        mot.setPower(power);
    };
}
