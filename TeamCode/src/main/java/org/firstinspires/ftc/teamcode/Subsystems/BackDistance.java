package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class BackDistance extends SubsystemBase {

    // Local objects and variables here

    private final DistanceSensor testDistance;
    private static double wallDistance;

    /** Place code here to initialize subsystem */
    public BackDistance() {

        testDistance =  RobotContainer.ActiveOpMode.hardwareMap.get(DistanceSensor.class, "rearDistance");
        wallDistance = 0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        wallDistance = 0.4*wallDistance+0.6*testDistance.getDistance(DistanceUnit.CM);

        RobotContainer.DBTelemetry.addData("Distance cm ", getDistance());
        RobotContainer.DBTelemetry.update();

    }

    // place special subsystem methods here

    public double getDistance(){

        return wallDistance;
    }

}