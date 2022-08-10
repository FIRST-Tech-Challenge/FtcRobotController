package org.firstinspires.ftc.teamcode.tests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.DriveSystem;

import java.util.EnumMap;

@Autonomous(name="Basic: Drive Test", group="Autonomous")
public class DriveTest extends OpMode {

    protected DriveSystem driveSystem;
    private EnumMap<DriveSystem.MotorNames, DcMotor> driveMap;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void init() {
        // Set up drive system
        driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name, hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));
        driveSystem.stopAndReset();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {
        double maxPower = 0.3;
        boolean driveComplete = false;
        if (!driveComplete) {
            driveComplete = driveSystem.driveToPosition(1000, DriveSystem.Direction.FORWARD, maxPower);
        }
        /*
        // Strafe left
        driveSystem.driveToPosition(1000, DriveSystem.Direction.LEFT, maxPower);
        // Strafe right
        driveSystem.driveToPosition(1000, DriveSystem.Direction.RIGHT, maxPower);
        // Drive backward
        driveSystem.driveToPosition(1000, DriveSystem.Direction.BACKWARD, maxPower);
         */
    }
}
