package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Alder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class AlderMecanumTeleOp /** Name your TeleOps different from other files or you get errors from having 2 files named the same */ extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fl = hardwareMap.dcMotor.get("fl"); //Remember ; at the end of your statements
        DcMotor fr = hardwareMap.dcMotor.get("fr"); //Remember ; at the end of your statements
        DcMotor bl = hardwareMap.dcMotor.get("bl"); //Remember ; at the end of your statements
        DcMotor br = hardwareMap.dcMotor.get("br"); //Remember ; at the end of your statements

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; //Remember ; at the end of your statements
            double x = gamepad1.left_stick_x; //Remember ; at the end of your statements
            double rx = gamepad1.right_stick_x; //Remember ; at the end of your statements

        }
    }
}
