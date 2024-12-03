package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class IntakeSpinnerHandler implements NKNComponent {

    CRServo servo;
    String servoName = "intakeServo";

    public IntakeSpinnerHandler() {

    }
    public void setServoPower (HandStates handStates){
        servo.setPower(handStates.power);
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
        setServoPower(HandStates.TRUE_STOP);
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

    public double getServoPower() {
        return servo.getPower();
    }

    public enum HandStates {
        GRIP(-0.3),
        RELEASE(0.25),
        REST(-0.07),
        TRUE_STOP(0);

        public final double power;

        HandStates(double power) {
            this.power = power;
        }
    }
}
