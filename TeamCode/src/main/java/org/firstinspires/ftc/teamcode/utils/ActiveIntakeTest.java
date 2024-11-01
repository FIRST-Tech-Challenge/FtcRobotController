package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Axon Intake Test", group="Subsystem Tests")
public class ActiveIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo intake = hardwareMap.get(CRServo.class, "crservo");
        AxonAbsolutePositionEncoder encoder = new AxonAbsolutePositionEncoder(hardwareMap, "encoder");


        //Change in angle = 5 degrees
        //Change in time = 0.2 seconds
        //If average angular velocity is less than 25 degrees per second, the sample is likely captured.

        waitForStart();
        long lastCheckedTime = System.currentTimeMillis();
        double lastCheckedPosition = Math.toDegrees(encoder.getAngle());

        while(!isStopRequested()){

            intake.setPower(gamepad1.left_stick_y);

            double angle = Math.toDegrees(encoder.getAngle());
            telemetry.addData("Current Servo Power: ", intake.getPower());
            telemetry.addData("Current Voltage: ",encoder.getVoltage());
            telemetry.addData("Current Servo Position: ",angle);

            if (Math.abs(intake.getPower()) > 0.1 && Math.abs(lastCheckedPosition - angle) < 5){
                telemetry.addLine("Capture Sample!");
            }

            long timeNow = System.currentTimeMillis();
            if (timeNow - lastCheckedTime > 200){ //0.2 seconds
                lastCheckedTime = timeNow;
                lastCheckedPosition = angle;
            }


            telemetry.update();
        }
    }
}
