package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class RightDistance extends SubsystemBase {

    // Local objects and variables here

    private final DistanceSensor testDistance;
    private static double rightDistance;

    /** Place code here to initialize subsystem */
    public RightDistance() {

        testDistance =  RobotContainer.ActiveOpMode.hardwareMap.get(DistanceSensor.class, "rightDistance");
        rightDistance = 0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // get current robot position
        Pose2d CurrentPose = RobotContainer.odometry.getCurrentPos();

        // only get distance measurements if robot in human player zone
        if (Math.abs(CurrentPose.getX())>1.0 && Math.abs(CurrentPose.getY())>0.5 )
        {
            rightDistance = 0.4*rightDistance+0.6*testDistance.getDistance(DistanceUnit.CM);
            RobotContainer.DBTelemetry.addData("RightDistance cm ", getDistance());
            RobotContainer.DBTelemetry.update();
        }

    }

    // place special subsystem methods here

    public double getDistance(){

        return rightDistance;
    }

}