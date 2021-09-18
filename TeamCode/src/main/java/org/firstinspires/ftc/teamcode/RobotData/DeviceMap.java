package org.firstinspires.ftc.teamcode.RobotData;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DeviceMap {
    public DcMotor FR, FL, BR, BL;

    ElapsedTime time = new ElapsedTime();

    public DeviceMap(){}

    public void init(HardwareMap hwMap){
        FR = hwMap.get(DcMotor.class, "FL");
        FL = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");
        BL = hwMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
