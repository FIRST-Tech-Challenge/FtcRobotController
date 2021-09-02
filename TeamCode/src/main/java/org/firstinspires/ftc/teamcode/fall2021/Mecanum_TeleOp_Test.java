package org.firstinspires.ftc.teamcode.fall2021;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Mecanum TeleOp Test", group = "Linear Opmode")
public class Mecanum_TeleOp_Test extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    double rotate = 0;
    double speed = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {
            double LFPower;
            double RFPower;
            double LBPower;
            double RBPower;

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;



            if(gamepad1.dpad_up){
                speed += 0.05;
                if(speed < 0){
                    speed = 0;
                }
            }
            if(gamepad1.dpad_down){
                speed -= 0.05;
                if(speed < 0){
                    speed = 0;
                }
            }

            LFPower  = Range.clip(speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(speed*(drive - rotate - strafe), -1.0, 1.0) ;

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.update();
        }

    }
}