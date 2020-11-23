package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="testing and hardware calibration", group = "UltimateGoal")
public class testingAndHardwareCalibration extends LinearOpMode {
    /*declare Opmode members, initialize some classes*/
    HardwareUltimateGoal robot          = new HardwareUltimateGoal();

    @Override
    public void runOpMode()
    {
        waitForStart();
        robot.init(hardwareMap);
        while (opModeIsActive()) {
            robot.turretLauncher.setPosition(0.5);
            robot.turretRotator.setPosition(0.5);
            robot.turretElevator.setPosition(0.5);

            sleep(5000);

            robot.turretLauncher.setPosition(0);
            robot.turretRotator.setPosition(0);
            robot.turretElevator.setPosition(0);

            sleep(10000);

            stop();
        }
    }
}
