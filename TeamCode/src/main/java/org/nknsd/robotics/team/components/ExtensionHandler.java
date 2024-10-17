package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class ExtensionHandler implements NKNComponent {
    private final String extenderName;
    private final boolean doInvertMotor;
    private final double motorPower;
    private DcMotor motor; // extender motor

    public enum ExtensionPositions {
        RESTING(0),
        HIGH_BASKET(20);

        final int position;

        ExtensionPositions(int position) {
            this.position = position;
        }
    }

    public ExtensionHandler(String extenderName, boolean doInvertMotor, double motorPower) {
        this.extenderName = extenderName;
        this.doInvertMotor = doInvertMotor;
        this.motorPower = motorPower;
    }
    
    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(extenderName);
        if (doInvertMotor) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        motor.setPower(motorPower);
        motor.setTargetPosition(ExtensionPositions.RESTING.position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        return "ExtensionHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Current Position", motor.getCurrentPosition());
        telemetry.addData("Target Position", motor.getTargetPosition());
    }

    public void gotoPosition(ExtensionPositions extensionPosition) {
        motor.setTargetPosition(extensionPosition.position);
    }
}
