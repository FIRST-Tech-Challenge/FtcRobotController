package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.

public class Hardware {
    final public ElapsedTime timePassed = new ElapsedTime();
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public void init(HardwareMap hardwareMap){
        try{
            frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setPower(0);
        } catch (Exception e){
            telemetry.add()
        }
    }


}
