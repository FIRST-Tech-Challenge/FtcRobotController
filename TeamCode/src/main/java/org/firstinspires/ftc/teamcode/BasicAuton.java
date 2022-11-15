package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;

class autonbot{
    public DcMotor BLeft = null;
    public DcMotor BRight = null;
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor Slider = null;
    public Servo Take = null;

    HardwareMap map = null;
    public void init(HardwareMap maps) {
        map = maps;
        BLeft = maps.dcMotor.get("bl");
        BRight = maps.dcMotor.get("br");
        FLeft = maps.dcMotor.get("fl");
        FRight = maps.dcMotor.get("fr");
        Take = maps.servo.get("grabber");
        Slider = maps.dcMotor.get("slider");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FRight.setDirection(DcMotorSimple.Direction.FORWARD);
        Slider.setDirection(DcMotorSimple.Direction.FORWARD);

        BLeft.setPower(0.0);
        BRight.setPower(0.0);
        FLeft.setPower(0.0);
        FRight.setPower(0.0);
        Slider.setPower(0.0);
        Take.setPosition(1.0);

        BLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

@Autonomous (name = "PowerPlaybot", group = "pp")

public class BasicAuton extends LinearOpMode{

    public void runOpMode(){

    }}

