package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.TeleOp.Intake.INTAKE;
import static org.firstinspires.ftc.teamcode.TeleOp.Intake.OUTTAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides {
    //establish left and right slide for gamepad
    public final DcMotor leftSlide, rightSlide;
    public final Gamepad gamepad2;
    //This determines where slides will stop depending on what the driver wants
    public static int HIGH = 450;
    public static int MID =358;
    public static int LOW =2;
    int position;
    //
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Slides(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;
        leftSlide = (DcMotor) hardwareMap.get("leftSlide");
        rightSlide = (DcMotor) hardwareMap.get("rightSlide");

        //The slides must be set to correct directions
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //brake
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //setting power, power can vary 0-1
        leftSlide.setPower(1);
        rightSlide.setPower(1);
        //
        rightSlide.setTargetPosition((int) INTAKE);
        leftSlide.setTargetPosition((int) OUTTAKE);


    }
}
