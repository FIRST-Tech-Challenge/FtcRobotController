package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "distance test")
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor distsense;

    @Override
    public void runOpMode() throws InterruptedException{

        distsense = hardwareMap.get(DistanceSensor.class,"distsense");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("deviceName",distsense.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", distsense.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", distsense.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", distsense.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", distsense.getDistance(DistanceUnit.INCH)));
            telemetry.addData("distance reading:", distsense.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}