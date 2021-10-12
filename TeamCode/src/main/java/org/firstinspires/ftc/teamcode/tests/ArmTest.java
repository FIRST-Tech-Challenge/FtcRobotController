package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmTest extends LinearOpMode {
    public void runOpMode(){
        DcMotor arm = hardwareMap.get(DcMotor.class, "intake");
        double targetPos = 0;

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){

            targetPos += gamepad1.left_stick_y * .75;

            if(gamepad1.a){
                targetPos = -280;
            }


            arm.setTargetPosition((int)targetPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            telemetry.addData("target pos", targetPos);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
