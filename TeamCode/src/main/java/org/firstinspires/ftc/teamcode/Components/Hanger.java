package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class Hanger extends RFMotor {
    double permaPower = 0.0;
    public Hanger(){
        super("hangerMotor", true);
    }
    public void setPermaPower(double p_permaPower){
        permaPower=p_permaPower;
    }
    @Override
    public void setPower(double p_power){
        setRawPower(p_power+permaPower);
    }
}
