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
    public final DcMotor Arm;
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
        Arm = (DcMotor) hardwareMap.get("ArmMotor");

        //The slides must be set to correct directions
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        //brake
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //setting power, power can vary 0-1
        Arm.setPower(0.5);
        //
        Arm.setTargetPosition(INTAKE);
    }
    public void teleOp() {
        if (gamepad2.dpad_up) outtake();
        else if (gamepad2.dpad_down) intake();
        else if (gamepad2.dpad_right) zeroPos();

        telemetry.addData("The right slide position in TICKS is: ", Arm.getCurrentPosition());

    }
    public void outtake() {
        Arm.setTargetPosition(HIGH);
    }
    public void intake() {
        //Use this for high chamber
        Arm.setTargetPosition(MID);
    }

    public void zeroPos() {
        Arm.setTargetPosition(INTAKE);

    }
}
