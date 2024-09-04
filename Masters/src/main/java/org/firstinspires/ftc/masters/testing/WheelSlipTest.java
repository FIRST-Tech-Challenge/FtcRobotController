package org.firstinspires.ftc.masters.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.components.ControllerMap;
import org.firstinspires.ftc.masters.components.DriveTrain;

@Config // Enables FTC Dashboard
@TeleOp(name = "Wheel Slip Test", group = "tests")
public class WheelSlipTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        new ControllerMap();
        while(true) {
            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);
        }
    }
}
