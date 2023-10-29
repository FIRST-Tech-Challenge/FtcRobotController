package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CyDogsSparky extends CyDogsChassis{

    private LinearOpMode myOpMode;
    public SpikeCam spikeCam;

    public CyDogsSparky(LinearOpMode currentOp) {
        super(currentOp);
        myOpMode = currentOp;
    }

    public void initializeSpikeCam(){
        spikeCam = new SpikeCam();
        spikeCam.initialize(myOpMode);
    }
}