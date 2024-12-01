package org.firstinspires.ftc.teamcode.oldCrap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "Test")
public class test extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                motor1.setPower(1);
            }else if(gamepad1.b){
                motor2.setPower(1);
            }else{
                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }
}
