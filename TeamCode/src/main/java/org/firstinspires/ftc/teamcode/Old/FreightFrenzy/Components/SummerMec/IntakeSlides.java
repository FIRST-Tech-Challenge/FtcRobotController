package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.SummerMec;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class IntakeSlides {
    RFMotor extendIntake;
    public IntakeSlides(){
        extendIntake = new RFMotor("turret_Rotation", DcMotor.RunMode.RUN_USING_ENCODER,true,545,0);
    }
    //sets velocity to extend to inputted position
    public void extendIntakeTo(double position){
        double distance = extendIntake.getCurrentPosition()-position;
        if(abs(distance)<10){
            extendIntake.setVelocity(0);
        }else{
            extendIntake.setVelocity(-5*distance+100*distance/abs(distance));
        }
    }
    public double getPosition(){
        return extendIntake.getCurrentPosition();
    }
}
