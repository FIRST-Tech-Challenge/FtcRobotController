package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
class ServoVals {
    public static double position = 0;
}

@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {
//    private SimpleServo servo;
    private Servo servo;
//    private CRServo servo;

    double multiplier = 1;
    boolean pressedA = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        motor = new Motor(hardwareMap, "m");
//        motor.setRunMode(Motor.RunMode.RawPower);

        servo = hardwareMap.servo.get("sv");

    }

    @Override
    public void loop() {

        servo.setPosition(ServoVals.position);

        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("Servo Direction", servo.getDirection());
//        telemetry.addData("Servo Position", servo.getCurrentPosition());
//        telemetry.addData("Servo Velocity", servo.getCorrectedVelocity());
//        telemetry.addData("Servo CPR", servo.getCPR());


    }
}