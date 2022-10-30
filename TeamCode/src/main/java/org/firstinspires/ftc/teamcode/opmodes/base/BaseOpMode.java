package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.params.DriveParams;

import java.util.EnumMap;

/**
 * Basic OpMode template
 */
public abstract class BaseOpMode extends OpMode {

    protected DriveSystem driveSystem;
    protected PixyCam pixycam;
    protected int step = 0;


    @Override
    public void init(){
        setDriveSystem();
        pixycam = hardwareMap.get(PixyCam.class, "pixy");
    }

    private void setDriveSystem() {
        // Instantiate Drive System motor map
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }

        // Instantiate IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, DriveParams.IMU);

        // Instantiate DriveSystem
        driveSystem = new DriveSystem(driveMap, imu);
    }

    protected boolean alignHeading(int sign) {
        int headingOffset = pixycam.headingOffset(sign);
        telemetry.addData("offset", headingOffset);
        if (headingOffset > 20) {
            driveSystem.turn(60, 0.5);
        } else if (headingOffset < -20) {
            driveSystem.turn(-60, 0.5);
        } else {
            driveSystem.setMotorPower(0);
            return true;
        }
        return false;
    }

    protected boolean alignDistance(int desiredWidth, boolean cone){
        int distanceOffset = pixycam.distanceOffset(desiredWidth);// find actual desired width
        if (distanceOffset > 10) {
            driveSystem.driveToPosition(100, DriveSystem.Direction.BACKWARD, 0.3);
        } else if (distanceOffset < -10) {
            driveSystem.driveToPosition(100, DriveSystem.Direction.FORWARD, 0.3);
        } else {
            driveSystem.setMotorPower(0);
            return true;
        }
        return false;
    }

    protected boolean align(int colorSignature, int desiredWidth, boolean cone){
        if(step == 0){
            if(alignHeading(colorSignature)){
                step++;
            }
        }
        if(step == 1){
            if(alignDistance(desiredWidth, cone)){
                return true;
            }
        }
        return false;
    }
}
