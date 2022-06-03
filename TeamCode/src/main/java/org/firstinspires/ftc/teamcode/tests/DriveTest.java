package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.DriveSystem;

import java.util.EnumMap;

public class DriveTest extends OpMode {

    protected DriveSystem driveSystem;

    @Override
    public void init() {
        // Set up drive system
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name, hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
    }

    @Override
    public void loop() {
        // Box w/right turns
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        driveSystem.turn(90, 1.0);
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        driveSystem.turn(90, 1.0);
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        driveSystem.turn(90, 1.0);
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        // Box w/left turns
        driveSystem.turn(-90, 1.0);
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        driveSystem.turn(-90, 1.0);
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        driveSystem.turn(-90, 1.0);
        driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, 1.0);
        driveSystem.turn(-90, 1.0);
        // Strafe left
        driveSystem.driveToPosition(2000, DriveSystem.Direction.LEFT, 1.0);
        // Strafe right
        driveSystem.driveToPosition(2000, DriveSystem.Direction.RIGHT, 1.0);
    }
}
