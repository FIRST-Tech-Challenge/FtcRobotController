package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "FINALCODE")
public class FlightLauncher extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor launcherMotor = hardwareMap.dcMotor.get("motor1");
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                launcherMotor.setPower(1);
            }
            else {
                launcherMotor.setPower(0);
            }
        }
    }
}