package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp;

public class TowerController extends TeleOp {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor screw = null;

    public TowerController (HardwareMap hardwareMap){
        screw = hardwareMap.get(DcMotor.class, "screw");

    }

    public void controlScrew() {
        screw = hardwareMap.get(DcMotor.class, "right_drive");
        screw.setDirection(DcMotor.Direction.FORWARD);

        
    }
}
