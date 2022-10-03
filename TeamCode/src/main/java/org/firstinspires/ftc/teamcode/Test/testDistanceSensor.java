package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.GBrobot;

@TeleOp
public class testDistanceSensor extends OpMode {
    GBrobot robot;

    @Override
    public void init() {
        robot = new GBrobot( this );

        telemetry.addLine( "Ready!" );
        telemetry.update( );
    }

    @Override
    public void loop() {
        telemetry.addData("Value of front distance sensor: " , robot.frontDist.getDistance());
        telemetry.update( );
    }

    /*
    private DistanceSensor front;
    private DistanceSensor back;
    private DistanceSensor left;
    private DistanceSensor right;

     @Override
    public void init() {

        front = hardwareMap.get(DistanceSensor.class, "front");
        back = hardwareMap.get(DistanceSensor.class, "front");
        left = hardwareMap.get(DistanceSensor.class, "front");
        right = hardwareMap.get(DistanceSensor.class, "front");
    }

   @Override
    public void loop() {
        telemetry.addLine("Value of front: " +String.valueOf(front_distSensor.getDistance()));
        telemetry.addLine("Value of back: " +String.valueOf(distSensors.getDistance("back")));
        telemetry.addLine("Value of left: " +String.valueOf(distSensors.getDistance("left")));
        telemetry.addLine("Value of right: " +String.valueOf(distSensors.getDistance("right")));
        telemetry.update();
    }*/
}