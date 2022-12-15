package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Control extends OpMode {
    public static final int TICKS_PER_INCH = (int)(537.6 * (2*Math.PI * 2));
    public boolean isStopRequested = false;
    Hraezlyr hraezlyr;
    enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
    }

    @Override
    public void init() {
        hraezlyr = new Hraezlyr(hardwareMap);
    }
    @Override
    public void stop(){
        if(isStopRequested) return;

        isStopRequested = true;

    }
    //538 ticks per rotation
    public void cascadeLift(Level zHeight){
        switch(zHeight){
            case GROUND:
                hraezlyr.cascadeMotor1.setTargetPosition(0);
                hraezlyr.cascadeMotor2.setTargetPosition(0);
                // all target positions are not accurate to the motor
            case LOW:
                hraezlyr.cascadeMotor1.setTargetPosition(1000);
                hraezlyr.cascadeMotor2.setTargetPosition(1000);
            case MEDIUM:
                hraezlyr.cascadeMotor1.setTargetPosition(2000);
                hraezlyr.cascadeMotor2.setTargetPosition(2000);
            case HIGH:
                hraezlyr.cascadeMotor1.setTargetPosition(3000);
                hraezlyr.cascadeMotor2.setTargetPosition(3000);
        }
    }
    public void cascadeLiftManual(double power){
        hraezlyr.cascadeMotor1.setPower(power);
        hraezlyr.cascadeMotor2.setPower(power);
    }
    public void sleep (long mill){
        try {
            Thread.sleep(mill);
        } catch (InterruptedException e) {
            e.printStackTrace();

        }
    }
    public double constrainAngle (double angle){
        return((angle % 360 + 360) % 360);
    }
}



