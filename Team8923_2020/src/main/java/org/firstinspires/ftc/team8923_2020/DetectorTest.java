package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "DetectorTest")
public abstract class DetectorTest extends MasterAutonomous{

    boolean isDetectingRings = true;

    public void runOpMode(){
        initAuto();
        alliance = Alliance.RED;
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.clear();

        while(opModeIsActive()){



        }

    }
}
