package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.pose;

public class FreeSpinOdoWheel extends OdometryWheel{
    DcMotor wheel;
    double ticksPerRev() {return 4096;}
    double radius (){ return  1.9;} //Centimeters

    public FreeSpinOdoWheel(pose offset, DcMotor wheel){
        super(offset);
        this.wheel = wheel;
    }

    @Override
    public long getWheelPosition() {
        return wheel.getCurrentPosition();
    }
}
