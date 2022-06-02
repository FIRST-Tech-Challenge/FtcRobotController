package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Tensorflow;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.EnumMap;

public abstract class BaseAutonomous extends BaseOpMode {
    Tensorflow tensorflow;
    Team currentTeam;

    public enum Team {
        RED, BLUE
    }

    public void init(BaseStateMachine.Team team) {
        super.init();
        
        EnumMap<DriveSystem.MotorNames, DcMotor> driveMap = new EnumMap<>(DriveSystem.MotorNames.class);
        for(DriveSystem.MotorNames name : DriveSystem.MotorNames.values()){
            driveMap.put(name,hardwareMap.get(DcMotor.class, name.toString()));
        }
        driveSystem = new DriveSystem(driveMap, hardwareMap.get(BNO055IMU.class, "imu"));

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        tensorflow = new Tensorflow(camName, tfodMonitorViewId);
        currentTeam = team;
    }
}