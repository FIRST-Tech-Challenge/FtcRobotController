package org.nknsd.teamcode.components.handlers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class ConveyorHandler implements NKNComponent {
    private CRServo lServo;
    private CRServo rServo;
    private final double servoPower = 1;

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        lServo = hardwareMap.crservo.get("lLift");
        rServo = hardwareMap.crservo.get("rLift");
        lServo.setDirection(DcMotorSimple.Direction.REVERSE);
        lServo.setPower(0);
        rServo.setPower(0);
        return true;
    }

    public void spinUp() {
        lServo.setPower(servoPower);
        rServo.setPower(servoPower);
    }

    public void spinStop() {
        lServo.setPower(0);
        rServo.setPower(0);
    }

    public void spinDown() {
        lServo.setPower(-servoPower);
        rServo.setPower(-servoPower);
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
        if (lServo.getPower() > 0) {
            telemetry.addLine("conveyor up");
        } else if (lServo.getPower() < 0) {
            telemetry.addLine("conveyor down");
        } else {
            telemetry.addLine("conveyor stopped");
        }
    }
}
