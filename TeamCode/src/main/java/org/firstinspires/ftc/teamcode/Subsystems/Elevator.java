package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private static final double ElevatorSpeedfast=0.95;
    private static final double ElevatorSpeedslow=0.6;
    private static final double Elevatoroff=0;
    public void init(HardwareMap hwMap){
    elevator =hwMap.get(DcMotor.class,"Elevator");
    elevator.setDirection(DcMotor.Direction.REVERSE);
    elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Elevatoroff() {
    elevator.setPower(Elevatoroff);
}
    public void ElevatorSpeedfast() {
    elevator.setPower(ElevatorSpeedfast);
}
    public void ElevatorSpeedslow() {
    elevator.setPower(ElevatorSpeedslow);
}
    public void Elevatorbackup() {
    elevator.setPower(-ElevatorSpeedslow);
}


}