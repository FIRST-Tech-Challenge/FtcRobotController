package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class Hanger extends RFMotor {
    double permaPower = 0.0;
    double max = 2000;
    boolean up = false;
    public Hanger(){
        super("hangerMotor", !isTeleop);
    }
    public void setPermaPower(double p_permaPower){
        permaPower=signum(p_permaPower)*min(abs(p_permaPower), 0.3);
        up = false;
    }
    @Override
    public void setPower(double p_power){
    if (this.getCurrentPosition() < max) {
            setRawPower(p_power + permaPower);
        }
    else{
        setRawPower(min(0,p_power));
    }
    up=false;
    }
    public void up(){
        this.setPosition(max,0);
        up = true;
    }
    public void update(){
        if(up){
            up();
        }
    }
}
