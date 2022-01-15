package org.firstinspires.ftc.teamcode.gen2_League;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test_ArmReader extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor liftControl = hardwareMap.get(DcMotor.class,"liftControl");


        waitForStart();

//        liftControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftControl.setTargetPosition(0);
//        liftControl.setPower(1.0);


        while (opModeIsActive()) {


            int tickCount = 560;


            if (gamepad1.left_stick_y > 0) {
                liftControl.setPower(-0.6);
            }
            if (gamepad1.left_stick_y < 0) {
                liftControl.setPower(0.6);
            }else {
                liftControl.setPower(0);
            }

            telemetry.addData("Position",liftControl.getCurrentPosition());
            telemetry.update();


        }


    }


}
