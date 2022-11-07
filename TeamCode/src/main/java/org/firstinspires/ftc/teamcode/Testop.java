package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Testop extends OpMode {
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;
    private double x;
    private double y;
    private double TurnRate;
    // 'armmotor' motorGrabinator;
    // 'pionhouder'motorConeHolder;
    // de dingen tussen hoge kommas moeten nog worden vervangen met de motornamen
    @Override
    public void init() {
        motorLeftBack = hardwareMap.dcMotor.get("linksachter");
        motorRightBack = hardwareMap.dcMotor.get("rechtsachter");
        motorLeftFront = hardwareMap.dcMotor.get("linksvoor");
        motorRightFront = hardwareMap.dcMotor.get("rechtsvoor");
        // motorGrabinator = hardwareMap.'armmotor'.get("armextender);
        // motorConeHolder = hardwareMap.'pionhouder'.get("mond");
        // de dingen tussen hoge kommas moeten nog worden vervangen met de motornamen

    }


    @Override
    public void start(){

    }


    @Override
    public void stop(){

    }

    @Override
    public void loop() {
        x=gamepad1.left_stick_x;
        y=gamepad1.left_stick_y;
        TurnRate= gamepad1.right_trigger-gamepad1.left_trigger;

        motorRightBack.setPower(x+y+TurnRate);
        motorLeftBack.setPower(x-y+TurnRate);
        motorRightFront.setPower(-x+y+TurnRate);
        motorLeftFront.setPower(-x-y+TurnRate);
        // motograbinator.setPower(gamepad2.right_bumper+gamepad2.left_bumper*-1);
        // motorconeholder.setPower(gamepad2.A+gamepad2.X*-1);
        // moet nog getest worden
    }


}