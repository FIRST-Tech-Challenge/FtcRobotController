package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "armLeft");
        waitForStart();
        if (opModeIsActive()) {

            // Pre-run
            while (opModeIsActive()) {
                if (gamepad1.y){
                        leftFrontDrive.setPower(-1);
                }
                else if (gamepad1.x) {
                    leftFrontDrive.setPower(-1);
                }
                else leftFrontDrive.setPower(0);
            }
        }
    }
}
