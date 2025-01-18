package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;

public class DeliveryAxonV1 {
    public Servo axon;
    public double savedAngle = 0.73;
    public HardwareMap hardwareMap;
    public RobotConstantsV1 constants = new RobotConstantsV1();

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        axon = hardwareMap.get(Servo.class, "deliveryServo");
    }

    public void setPosition(double position){
        axon.setPosition(position);
        saveAngle(position);
    }

    public double getPosition(){
        return this.savedAngle;
    }

    public double getCurrentPosition(){
        return axon.getPosition();
    }

    public void saveAngle(double angle){
        this.savedAngle = angle;
    }

    public void goToSavedAngle(){
        if(isAngleSaved()){
            setPosition(savedAngle);
        }
        else {
            setPosition(constants.DELIVERY_HIGH_BAR);
        }
    }

    public boolean isAngleSaved(){
        if(savedAngle == 0.0){
            return false;
        }
        else {
            return true;
        }
    }

}
