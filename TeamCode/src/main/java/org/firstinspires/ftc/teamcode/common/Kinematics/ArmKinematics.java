package org.firstinspires.ftc.teamcode.common.Kinematics;

import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

public class ArmKinematics {
    protected Constants constants = new Constants();


    //current positions
    protected double currentBase = 0;
    protected double currentTop = 0;

    public ArmKinematics(){

    }

    public void setCurrentPosition(){
        currentBase = 10;
        currentTop = 10;
    }
}
