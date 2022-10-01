package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
public class MecanumDriveBase {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor lf= null;
    public DcMotor lb= null;
    public DcMotor rb= null;
    public DcMotor rf= null;

    public void MecanumController(OpMode opMode, HardwareMap hardwareMap){
        rb  = hardwareMap.get(DcMotor.class, "rb");
        rf  = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        lf  = hardwareMap.get(DcMotor.class, "lf");
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
    }
}

