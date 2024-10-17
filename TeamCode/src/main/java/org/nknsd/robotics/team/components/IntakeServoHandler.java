package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class IntakeServoHandler implements NKNComponent {

    CRServo servo;
    String servoName;

    public IntakeServoHandler(String servoName) {
        this.servoName = servoName;
    }
    void setServoPower (double power){
        servo.setPower(power);
    }



    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        servo = hardwareMap.crservo.get(servoName);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        String servoString = "[Servo Power" + servo.getPower() + "]";
        telemetry.addData("intakeServo", servoString);
    }

}