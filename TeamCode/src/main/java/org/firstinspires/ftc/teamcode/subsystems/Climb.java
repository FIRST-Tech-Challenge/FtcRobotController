package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climb {
    private DcMotor climb;
//    private Encoder climbEncoder;

    public Climb(HardwareMap hw, String climbName)
    {
        climb = hw.get(DcMotor.class, climbName);

    }

    public void moveClimb(double power){

        climb.setPower(power);
    }


}
