package org.nknsd.teamcode.components.handlers.intakeTurret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class IntakeHandler implements NKNComponent {

    private DcMotor motor;

    private final int maxPower = 1;



    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get("intakeMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        return "";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake forward ", intakeForward);
        telemetry.addData("Intake reverse ", intakeReverse);
    }

    boolean intakeForward = false;
    boolean intakeReverse = false;

    public void startIntake(){
        if(!intakeForward){
            motor.setPower(maxPower);
            intakeForward = true;
            intakeReverse = false;
        } else {
            stopIntake();
        }



    }

    public void stopIntake() {
        intakeForward = false;
        intakeReverse = false;
        motor.setPower(0);
    }

    public void reverseIntake() {
        if(!intakeReverse) {
            motor.setPower(-maxPower);
            intakeReverse = true;
            intakeForward = false;
        } else {
            stopIntake();
        }

    }
}
