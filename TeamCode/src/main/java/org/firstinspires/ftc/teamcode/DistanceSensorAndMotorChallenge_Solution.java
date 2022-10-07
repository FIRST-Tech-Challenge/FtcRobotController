package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class DistanceSensorAndMotorChallenge_Solution extends LinearOpMode{
    // Declare a single motor and the distance sensor
    private DcMotor Motor0 = null;
    private DistanceSensor sensorRange;

    @Override
    public void runOpMode(){
        // Hardware maps
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");

        // Set the direction of the motor
        Motor0.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">>>", "Press start to continue");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            // capture the distance measured by the sensor, then scale it down so it fits within the 0.0 -> 1.0 range
            // e.g. 2.0 on the sensor = 1.0 on the motor, 1.0 on the sensor = 0.5 on the motor etc.
            double measuredDistance = sensorRange.getDistance(DistanceUnit.METER);
            double scaledPower = Range.scale(measuredDistance, 0.0, 2.0, 0.0, 1.0);

            // Clip the power just in case it goes out of bounds
            scaledPower = Range.clip(scaledPower, 0.0, 1.0);

            //Set the motor to the power
            Motor0.setPower(scaledPower);


            telemetry.addData("range", String.format("%.01f m", measuredDistance));
            telemetry.addData("Motor0Power", "(%.2f", scaledPower);
            telemetry.update();
        }
    }
}
