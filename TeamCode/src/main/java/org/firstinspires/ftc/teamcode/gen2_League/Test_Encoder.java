package org.firstinspires.ftc.teamcode.gen2_League;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(preselectTeleOp = "Beta_TeleOp")
public class Test_Encoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRightMotor");


        waitForStart();
        if (opModeIsActive()) {

            int tickCount = 1213;

            int halfTurn = tickCount/2;

            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setTargetPosition(halfTurn);

            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setPower(1);


        }


    }


}
