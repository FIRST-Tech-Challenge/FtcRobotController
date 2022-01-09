package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Outtake Test")
public class TestMotor extends LinearOpMode {
    private DcMotor motorOuttake;

    @Override
    public void runOpMode() throws InterruptedException {
        motorOuttake = hardwareMap.dcMotor.get("outtake");
        
        motorOuttake.setTargetPosition(0);
        motorOuttake.setPower(0);
        motorOuttake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                motorOuttake.setPower(.5);
                motorOuttake.setTargetPosition(800);
                
                if(motorOuttake.getCurrentPosition()<800){
                    while(motorOuttake.getCurrentPosition()<800);
                }
                else{
                    while(motorOuttake.getCurrentPosition()>800);   
                }

                motorOuttake.setPower(0);
            }
            if (gamepad1.right_bumper) {
                motorOuttake.setPower(.5);
                motorOuttake.setTargetPosition(0);
                
                if(motorOuttake.getCurrentPosition()>0){
                    while(motorOuttake.getCurrentPosition()>0);
                }
                else{
                    while(motorOuttake.getCurrentPosition()<0);   
                }

                motorOuttake.setPower(0);
            }
        }
    }
}
