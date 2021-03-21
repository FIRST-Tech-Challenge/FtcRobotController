package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.MasterTeleOp;

@TeleOp(name = "LauncherTest", group = "TeleOp")
public class LauncherTest extends MasterTeleOp {

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            driveLauncherWithController();
            
            telemetry.addData("Launcher RPM", getMotorSpeed(motorLauncher, 100));

            idle();
        }
    }
}