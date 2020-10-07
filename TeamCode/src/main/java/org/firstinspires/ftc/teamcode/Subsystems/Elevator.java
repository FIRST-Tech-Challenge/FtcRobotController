package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static android.icu.util.ULocale.getName;

public class Elevator {
    //
    //define hardware
    public DcMotor elevator = null;
    private DigitalChannel switch1 = null;
    private DigitalChannel switch2 = null;

    //constants

    private static final double ElevatorSpeedfast=0.5;
    private static final double Elevatorspeedslow=0.25;

    public void init(HardwareMap hwMap){
    elevator =hwMap.get(DcMotor.class,"Elevator");








    }



}
