package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class FrontDistance extends SubsystemBase {

    // Local objects and variables here

    private final DistanceSensor testDistance;
    private static double frontDistance;

    /** Place code here to initialize subsystem */
    public FrontDistance() {

        testDistance =  RobotContainer.ActiveOpMode.hardwareMap.get(DistanceSensor.class, "frontDistance");
        frontDistance = 0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // get current robot position
        Pose2d CurrentPose = RobotContainer.odometry.getCurrentPos();

        // only get distance measurements if robot in sub dropoff zone
        if (CurrentPose.getX()>=-0.6 && CurrentPose.getX()<=0.6)
        {
            frontDistance = 0.4*frontDistance+0.6*testDistance.getDistance(DistanceUnit.CM);
            RobotContainer.DBTelemetry.addData("FrontDistance cm ", getDistance());
            RobotContainer.DBTelemetry.update();
        }

    }

    // place special subsystem methods here

    public double getDistance(){

        return frontDistance;
    }

}