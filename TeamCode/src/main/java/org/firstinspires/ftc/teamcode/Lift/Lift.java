package org.firstinspires.ftc.teamcode.Lift;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Utils.MotionProfile;

public class Lift {
    HardwareMap hardwareMap;
    FeedForward feedForward;
    PID pid;
    DcMotorEx liftMotorLeft;
    DcMotorEx liftMotorRight;
    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftMotorRight");

    }



}
