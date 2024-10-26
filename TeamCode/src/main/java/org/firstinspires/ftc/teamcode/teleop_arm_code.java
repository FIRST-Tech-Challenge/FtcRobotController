package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay23_24.TemplateJanx;

@TeleOp(name = "teleop arm code")
public class teleop_arm_code extends LinearOpMode {
    //replace Template with your class name
    //initiate motors
    // private motorType nameOfMotor
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo claw;
    @Override
    public void runOpMode() {
        TemplateJanx janx = new TemplateJanx(hardwareMap);
        janx.basicArmInit("arm_left", "arm_right");
        //link motors to config
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //call functions here
                arm(gamepad2.left_stick_y);
                servo();
            }
        }
    }
    //write functions here
    private void arm(double LSY){
        if(LSY > 0) {
         leftMotor.setPower(1);
         rightMotor.setPower(1);
        }
        else if(LSY< 0){
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);
        }
    }
    private void servo(){
        if(gamepad2.a)
        {
            claw.setPosition(1);
        }
        if(gamepad2.b)
        {
            claw.setPosition(-1);
        }
    }
}