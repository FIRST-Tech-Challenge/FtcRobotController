package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testeree extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor tester = hardwareMap.dcMotor.get("tester");
        DcMotor motot1 = hardwareMap.dcMotor.get("motot1");



        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            tester.setPower(gamepad1.right_trigger);
            motot1.setPower(gamepad1.left_trigger);
        }
    }


}
