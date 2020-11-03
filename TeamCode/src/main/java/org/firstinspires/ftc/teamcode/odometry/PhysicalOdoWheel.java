package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.pose;

public class PhysicalOdoWheel extends OdometryWheel{
    DcMotor wheel;

    public PhysicalOdoWheel(pose offset, DcMotor wheel){
        super(offset);
        this.wheel = wheel;
    }

    @Override
    public long getWheelPosition() {
        return wheel.getCurrentPosition();
    }
}
