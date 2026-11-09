package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferDriver {
    
    private MotorSpeedController transferMotor;
    private MotorSpeedController feedMotor;

    // initilizes both the treansfer and feed motors in the MotorSpeedController class
    public TransferDriver(HardwareMap hm, String transferMotorName, String feedMotorName){
        this.transferMotor = new MotorSpeedController(hm, transferMotorName, 312);
        this.feedMotor = new MotorSpeedController(hm, feedMotorName, 312);
        this.feedMotor.setDirection(-1);
    }

    // spins motors in directions so they get intaked and held in the transfer system, but not put in the turret
    public void collect(){
        this.transferMotor.setRPM(150);
        this.feedMotor.setRPM(312);
    }

    // sets motors in transfer to remove balls from system
    public void reverse(){
        this.transferMotor.setRPM(-150);
        this.feedMotor.setRPM(312);
    }

    // makes all motors in transfer system to turn off
    public void off(){
        this.transferMotor.setRPM(0);
        this.feedMotor.setRPM(0);
    }

    // makes the fly wheel change direction its spinning to load a ball from transfer into the turret
    public void loadTurret(){
        this.transferMotor.setRPM(312);
        this.feedMotor.setRPM(-312);
    }

    // returns number of balls in transfer system
    //TODO finish method using camera vision (likely Husky lenses) 
    public int getNumberOfBalls(){
        return -1;
    }

    // tells driver if balls are in tranfser system
    private boolean ballReady(){
        if(getNumberOfBalls() >= 1){
            return true;
        }else {
            return false;
        }
    }

}
