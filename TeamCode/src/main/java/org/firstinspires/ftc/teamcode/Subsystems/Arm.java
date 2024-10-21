package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    DcMotor armMotor;
    public int REST = 0;
    public int INTAKE = 100;
    public int OUTTAKE = 200;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;

    public Arm (OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.gamepad = opMode.gamepad2;
        this.hardwareMap = opMode.hardwareMap;

        armMotor = (DcMotor) hardwareMap.get("ArmMotor");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }
    public void teleOp() {
        if (gamepad2.dpad_left) goOuttakePos();
        else if (gamepad2.dpad_right) goIntakePos();
        else rest();
    }
    public void goOuttakePos() {
        armMotor.setTargetPosition(OUTTAKE);
        periodic();
    }
    public void goIntakePos() {
        armMotor.setTargetPosition(INTAKE);
        periodic();
    }
    public void rest() {
        armMotor.setTargetPosition(REST);
        periodic();
    }
    public void periodic() {
        telemetry.update();
        telemetry.addData("Arm position is: ", (armMotor.getCurrentPosition()));
    }
}
