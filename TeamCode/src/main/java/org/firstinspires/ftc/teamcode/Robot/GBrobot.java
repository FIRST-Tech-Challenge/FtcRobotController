package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.distance_sensor;

public class GBrobot {
    public OpMode opMode;
    public HardwareMap hardwareMap;
    Telemetry telemetry;

    public distance_sensor frontDist;

    public GBrobot( OpMode op ) {

        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // initialize objects/classes
        frontDist = new distance_sensor( hardwareMap, "front", opMode.telemetry );
    }
}
