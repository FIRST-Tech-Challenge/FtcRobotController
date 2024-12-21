package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.auto.CheckDriveStraight.isWithinTolerance;
import static org.firstinspires.ftc.teamcode.auto.CheckDriveStraight.turnToCorrectSide;

// class for handling the IMU and its calculations
public class InterfaceErrorIMU {
    private final String name;
    private double rotationLeft = 0;
    private double power = 0.075;
    private IMU imu;

    public InterfaceErrorIMU(String name){
        this.name = name;
    }


    public String getName() {
        return name;
    }
    public void setImu(IMU imu){
        this.imu = imu;
    }
    public IMU getImu() {
        return imu;
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    // so now 180 is straight ahead and directly behind is 0 or 360
    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;
    }
    //get if it not facing the target with a tolerance
    public boolean notFacing(int target){
        rotationLeft=0;
        boolean correctDriction=false;
        int cur = (int)getYaw();
        if (!isWithinTolerance(cur, target, 3)){
            rotationLeft+=power;
            correctDriction=true;
        }
        if (!isWithinTolerance(cur, target, 20)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 40)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 80)){
            rotationLeft+=power;
        }
        return correctDriction;
    }
    //returns the power to the left side wheel to turn to correct
    public double getRotationLeftPower(int target) {
        return rotationLeft*(turnToCorrectSide(getYaw(), target) ? 1 : -1);
    }
}
