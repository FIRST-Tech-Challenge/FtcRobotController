package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tutorial")
public class Tutorial extends OpMode {

    public final static String strMotorName = "机器人";

    private double motorDefaultPower = 0.5f;
//    Vector2 moveDirection = new Vector2(0, -1);
    DcMotor motor;//定义电机


    @Override
    public void init() {
        //在DCMotor的硬件映射中获取motor的电机
        motor = hardwareMap.get(DcMotor.class, "motor");


    }

    @Override
    public void loop() {
        telemetry.addData("Initialize", " is Sucess!");
        telemetry.addData("ftc", ": first");
        if(gamepad1.a){
            telemetry.addData("dumbo", " it works!");
            telemetry.update();
        }

        if(null != motor){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(motorDefaultPower);
        }

    }
}
