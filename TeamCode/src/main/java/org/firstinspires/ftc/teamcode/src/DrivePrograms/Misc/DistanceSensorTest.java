package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;


import static org.firstinspires.ftc.teamcode.src.Utills.MiscUtills.getStackTraceAsString;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;

@TeleOp(name = "DistanceSensorTest")

public class DistanceSensorTest extends TeleopTemplate {


    DistanceSensor distanceSensor;
    RevBlinkinLedDriver.BlinkinPattern defaultColor;
    RevBlinkinLedDriver.BlinkinPattern currentColor;


    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        try {

            leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
            defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            currentColor = defaultColor;
            leds.setPattern(defaultColor);

            distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {

                leds.setPattern(currentColor);
                intake.setMotorPower(gamepad2.left_trigger - gamepad2.right_trigger);
                telemetry.addData("distance:", distanceSensor.getDistance(DistanceUnit.CM));

                if (distanceSensor.getDistance(DistanceUnit.CM) < 8.5) {
                    telemetry.addData("item detected", "");
                    currentColor = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
                } else {
                    currentColor = defaultColor;
                }

                telemetry.update();

            }

        } catch (Exception e) {
            String s = getStackTraceAsString(e);
            telemetry.addData("Exception", s);
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                Thread.sleep(1);
            }
        }
    }
}
