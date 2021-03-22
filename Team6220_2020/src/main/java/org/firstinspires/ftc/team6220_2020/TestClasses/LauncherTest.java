package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.MasterOpMode;
import org.firstinspires.ftc.team6220_2020.MasterTeleOp;

@TeleOp(name = "LauncherTestWithRPM", group = "TeleOp")
public class LauncherTest extends MasterTeleOp {

    @Override
    public void runOpMode() {

        telemetry.addLine("Launcher RPM");

        Initialize();

        waitForStart();

        while (opModeIsActive()) {

            driveLauncherWithController();

            telemetry.addData("Launcher RPM", getMotorSpeed(motorLauncher, 200));
            telemetry.update();

            idle();
        }
    }
}