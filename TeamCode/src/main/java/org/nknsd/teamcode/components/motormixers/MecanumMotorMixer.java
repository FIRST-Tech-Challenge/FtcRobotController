package org.nknsd.teamcode.components.motormixers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class MecanumMotorMixer implements NKNComponent {

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private boolean enabled = true;
    private double blPower;
    private double flPower;
    private double frPower;
    private double brPower;

    public void setPowers(double[] powers) {
        double magnitude;
        double x = -powers[0];
        double y = powers[1];
        double turning = -powers[2];
        magnitude = Math.abs(y) + Math.abs(x) + Math.abs(turning);
        if (magnitude > 1) {
            y = y / magnitude;
            x = x / magnitude;
            turning = turning / magnitude;
        }
        flPower = y - x + turning;
        frPower = y + x - turning;
        blPower = y + x + turning;
        brPower = y - x - turning;

        if (enabled) {
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
        } else {
            flMotor.setPower(0);
            frMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
        }
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        flMotor = hardwareMap.dcMotor.get("FL");
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor = hardwareMap.dcMotor.get("FR");
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor = hardwareMap.dcMotor.get("BL");
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor = hardwareMap.dcMotor.get("BR");
        //brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        telemetry.addData("fl", flPower);
        telemetry.addData("fr", frPower);
        telemetry.addData("bl", blPower);
        telemetry.addData("br", brPower);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
