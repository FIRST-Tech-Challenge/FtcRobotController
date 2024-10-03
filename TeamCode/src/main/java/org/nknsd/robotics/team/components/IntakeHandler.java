package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class IntakeHandler implements NKNComponent {
    private final String motorName;
    private final String servoName;
    private final boolean doInvertMotor;

    private DcMotor motor; private CRServo servo;

    public enum HandStates {
        GRIP(1),
        RELEASE(-1),
        REST(0);

        public final double power;

        HandStates(double power) {
            this.power = power;
        }
    }

    public IntakeHandler(String motorName, String servoName, boolean invertMotor) {
        this.motorName = motorName;
        this.servoName = servoName;
        this.doInvertMotor = invertMotor;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(motorName);
        if (doInvertMotor) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        servo = hardwareMap.crservo.get(servoName);

        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public String getName() {return "IntakeHandler";}

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {}

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Arm Position", motor.getCurrentPosition());
        telemetry.addData("Arm Target", motor.getTargetPosition());
    }

    public void controlHand(HandStates targetState) {
        servo.setPower(targetState.power);
    }

}
