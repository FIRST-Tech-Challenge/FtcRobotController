package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class SideDistance extends SubsystemBase {

    // Local objects and variables here

    private final DistanceSensor testDistance;
    private static double rightWallDistance;

    /** Place code here to initialize subsystem */
    public SideDistance() {

        testDistance =  RobotContainer.ActiveOpMode.hardwareMap.get(DistanceSensor.class, "rightDistance");
        rightWallDistance = 0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        rightWallDistance = 0.4*rightWallDistance+0.6*testDistance.getDistance(DistanceUnit.CM);

        RobotContainer.DBTelemetry.addData("RightDistance cm ", getDistance());
        RobotContainer.DBTelemetry.update();

    }

    // place special subsystem methods here

    public double getDistance(){

        return rightWallDistance;
    }

}