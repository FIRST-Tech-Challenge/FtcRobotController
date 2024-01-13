package org.firstinspires.ftc.teamcode.alex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled

public class testOpModee extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor motorTest;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    private Servo servoTest;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class,"imu");
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double tgtPower=0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tgtPower=-this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("target Power", tgtPower);
            telemetry.addData("Gamepad1.yPosition", gamepad1.left_stick_y);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
