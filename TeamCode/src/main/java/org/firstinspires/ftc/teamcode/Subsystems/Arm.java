package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    public final DcMotor arm;
    public final Gamepad gamepad2;
    private static int HIGH = 50;
    public static int MID = 10;
    public static int INTAKE = 0;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Arm(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;
        arm = (DcMotor) hardwareMap.get("ArmMotor");

        //The slides must be set to correct directions
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        //brake
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //setting power, power can vary 0-1
        arm.setPower(0.5);
        //
        arm.setTargetPosition(INTAKE);
    }
    public void teleOp() {
        if (gamepad2.dpad_up) outtake();
        else if (gamepad2.dpad_down) intake();
        else if (gamepad2.dpad_right) zeroPos();
        else manual();

        telemetry.addData("The right slide position in TICKS is: ", arm.getCurrentPosition());

    }
    public void outtake() {
        arm.setTargetPosition(HIGH);
    }
    public void intake() {
        //Use this for high chamber
        arm.setTargetPosition(MID);
    }

    public void zeroPos() {
        arm.setTargetPosition(INTAKE);
    }
    public void manual() {
        while (gamepad2.left_stick_y >= 0.5 && gamepad2.left_stick_y <= -0.5) {
            arm.setPower(-gamepad2.left_stick_y);
        }
    }
}
