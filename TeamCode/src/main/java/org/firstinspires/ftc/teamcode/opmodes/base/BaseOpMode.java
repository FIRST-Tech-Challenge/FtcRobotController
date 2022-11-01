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
    int distanceOffset;


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

    protected boolean alignDistance(int colorSignature, int desiredWidth){
        distanceOffset = pixycam.distanceOffset(colorSignature, desiredWidth);// find actual desired width
        telemetry.addData("offset", distanceOffset);
        if (distanceOffset > 50) {
            telemetry.addData("driving backwards", 0);
            driveSystem.drive(0, 0, 0.3f);
            return false;
        } else if (distanceOffset < -50) {
            telemetry.addData("driving forward", 0);
            driveSystem.drive(0, 0, 0.3f);
            return false;
        } else {
            telemetry.addData("stopping", 0);
            driveSystem.setMotorPower(0);
            return true;
        }
    }

    protected boolean align(int colorSignature, int desiredWidth){
        if(step == 0){
            if(alignHeading(colorSignature)){
                step++;
            }
        }
        if(step == 1){
            if(alignDistance(colorSignature, desiredWidth)){
                step = 0;
                return true;
            }
        }
        return false;
    }
}
