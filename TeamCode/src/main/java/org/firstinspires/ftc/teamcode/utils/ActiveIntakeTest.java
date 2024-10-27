package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name="Axon Intake Test", group="Subsystem Tests")
public class ActiveIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo intake = hardwareMap.get(CRServo.class, "crservo");
        AxonAbsolutePositionEncoder encoder = new AxonAbsolutePositionEncoder(hardwareMap, "encoder");

        waitForStart();

        while(!isStopRequested()){

            intake.setPower(gamepad1.left_stick_y);


            telemetry.addData("Current Servo Power: ", intake.getPower());
            telemetry.addData("Current Voltage: ",encoder.getVoltage());
            telemetry.addData("Current Servo Position: ",Math.toDegrees(encoder.getAngle()));

            telemetry.update();
        }
    }
}
