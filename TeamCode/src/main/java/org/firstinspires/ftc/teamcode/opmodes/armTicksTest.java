package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "armTicksTest")
public class armTicksTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "armLeft"); // port 0
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "armRight"); // port 3
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                /// Telemetry
                telemetry.addData("Right: ", rightDrive.getCurrentPosition());
                telemetry.addData("Left: ",  leftDrive.getCurrentPosition());

                telemetry.update();
            }
        }
    }
}
