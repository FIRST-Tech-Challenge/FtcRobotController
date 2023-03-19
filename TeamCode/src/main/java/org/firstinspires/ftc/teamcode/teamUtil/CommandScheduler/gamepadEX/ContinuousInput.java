package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Trigger;

import java.util.function.DoubleSupplier;

public class ContinuousInput {
    private final DoubleSupplier input;
    private double value;

    private double maxValue;
    private double deadZone;
    private boolean curve;

    public ContinuousInput(DoubleSupplier input){
        this.input = input;
        this.maxValue = 1;
        this.deadZone = 0;
        this.value = 0;
        this.curve = true;
    }

    public double getValue(){
        if(Math.abs(input.getAsDouble()) > maxValue){
            maxValue = Math.abs(input.getAsDouble());
        }
        value = input.getAsDouble()/maxValue;
        if(Math.abs(value) <= deadZone){
            value = 0;
        }
        if(curve){
            value *= Math.signum(value)*value;
        }
        return value;
    }
    
    /**
     *
     * @param percentage range from 0 - 1
     */
    public void applyDeadZone(double percentage){
        if(percentage < 0){
            percentage = 0;
        }
        if(percentage > 1){
            percentage = 1;
        }
        this.deadZone = percentage;
    }
    
    /**
     *
     * @param curve enabling or disabling the curving of a continuous input with a deadzone enabled.
     */
    public void applyCurve(boolean curve){
        this.curve = curve;
    }
    
    public Trigger thresholdTrigger(double threshold){
        return new Trigger(() -> Math.abs(getValue()) >= threshold);
    }

}
