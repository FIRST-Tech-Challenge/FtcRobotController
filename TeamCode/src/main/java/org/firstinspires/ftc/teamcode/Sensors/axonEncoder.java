package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class axonEncoder{

    private AnalogInput analog;

    private double angle = 0;
    private double currentAngle;
    private double lastAngle;

    private int count = 0;

    public axonEncoder(AnalogInput analog) {
        this.analog = analog;
    }

    public double getVoltage(){
        return analog.getVoltage();
    }
    public double update() {
        angle = getVoltage()/3.3*360;
        return angle;
    }//0 to 360 then wraps around back to 0

    public double updateAngle(){
        currentAngle = update();
        if(Math.abs(currentAngle - lastAngle) > 180){
            count += Math.signum(lastAngle - currentAngle);
        }
        lastAngle = currentAngle;
        return count * 360 + currentAngle;
    }//-inf to pos inf

    public double getAngleWrapped(){
        return updateAngle();
    }

}
