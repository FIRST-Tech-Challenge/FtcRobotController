package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Blue Caroseul Side Test")
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor distsense;

    @Override
    public void runOpMode() throws InterruptedException{

        distsense = hardwareMap.get(DistanceSensor.class,"distsense");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("distance reading:", distsense.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}