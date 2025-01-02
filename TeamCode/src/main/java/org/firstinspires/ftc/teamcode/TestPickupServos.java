package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test PickupServos")
public class TestPickupServos extends OpMode {

    private Servo axis1 = null;
    private Servo axis2 = null;
    private CRServo left = null;
    private CRServo right = null;
    Gamepad prevGamepad1 = new Gamepad();
    Gamepad currGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();
    Gamepad currGamepad2 = new Gamepad();

    double axis1Step = .005;
    double axis2Step = .005;

    private Telemetry.Item telAxis1Pos = null;
    private Telemetry.Item telAxis2Pos = null;

    @Override
    public void init() {
        axis1 = hardwareMap.get(Servo.class, "axis1");
        axis2 = hardwareMap.get(Servo.class, "axis2");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telAxis1Pos = telemetry.addData("Axis1 Pos:", "");
        telAxis2Pos = telemetry.addData("Axis2 Pos:", "");

    }

    @Override
    public void start() {
        axis1.setPosition(0.5);
        axis2.setPosition(0.5);

    }

    @Override
    public void loop() {
        currGamepad1.copy(gamepad1);
        currGamepad2.copy(gamepad2);

        //axis1 increase
        if(currGamepad1.dpad_up && !prevGamepad1.dpad_up) {
            double axis1Pos = axis1.getPosition();
            axis1.setPosition(Math.min(axis1Pos+axis1Step, 1.0));
        }

        //axis1 decrease
        if(currGamepad1.dpad_down && !prevGamepad1.dpad_down) {
            double axis1Pos = axis1.getPosition();
            axis1.setPosition(Math.max(axis1Pos-axis1Step, 0.0));
        }

        //axis2 increase
        if(currGamepad1.dpad_left && !prevGamepad1.dpad_left) {
            double axis2Pos = axis2.getPosition();
            axis2.setPosition(Math.min(axis2Pos+axis2Step, 1.0));
        }

        //axis2 decrease
        if(currGamepad1.dpad_right && !prevGamepad1.dpad_right) {
            double axis2Pos = axis2.getPosition();
            axis2.setPosition(Math.max(axis2Pos-axis2Step, 0.0));
        }

        //intake
        if(currGamepad1.right_bumper && !prevGamepad1.left_bumper) {
            left.setPower(-1.0);
            right.setPower(1.0);
        }
        //expell
        else if(currGamepad1.left_bumper && !prevGamepad1.right_bumper) {
            left.setPower(1.0);
            right.setPower(-1.0);
        }
        //stop
        else if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper) {
            left.setPower(0.0);
            right.setPower(0.0);
        }

        //left motor only intake
        if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper && currGamepad1.y && !prevGamepad1.a) {
            left.setPower(-1.0);
        }
        //expell
        else if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper && !currGamepad1.y && prevGamepad1.a) {
            left.setPower(1.0);
        }
        //stop
        else if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper && !currGamepad1.y && !prevGamepad1.a) {
            left.setPower(0.0);
        }

        //right motor only intake
        if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper && currGamepad1.x && !prevGamepad1.b) {
            right.setPower(-1.0);
        }
        //expell
        else if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper && !currGamepad1.x && prevGamepad1.b) {
            right.setPower(1.0);
        }
        //stop
        else if(!currGamepad1.left_bumper && !prevGamepad1.right_bumper && !currGamepad1.x && !prevGamepad1.b) {
            right.setPower(0.0);
        }

        telAxis1Pos.setValue("%.6f", axis1.getPosition());
        telAxis2Pos.setValue("%.6f", axis2.getPosition());

        prevGamepad1.copy(currGamepad1);
        prevGamepad2.copy(currGamepad2);
    }
}