package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class teleinit extends New_Super_Class {
    public Drive drive;
    public Intake intake;
    public Lift lift;





    public double start_time;

    @Override
    public void inits() {
        drive = new Drive(this);
        intake = new Intake(this);
        lift = new Lift(this);
        while(!isStarted() && !isStopRequested() && !opModeIsActive()) {
            lift.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
