package org.firstinspires.ftc.teamcode.Components.SummerMec;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class IntakeSlides {
    RFMotor extendIntake;
    public IntakeSlides(){
        extendIntake = new RFMotor("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,true,545,0);
    }
    public void extendIntakeTo(double position){
        double distance = extendIntake.getCurrentPosition()-position;
        if(distance<10){
            extendIntake.setVelocity(0);
        }else{
            extendIntake.setVelocity(-abs(5*distance+100));
        }
    }
    public double getPosition(){
        return extendIntake.getCurrentPosition();
    }
}
