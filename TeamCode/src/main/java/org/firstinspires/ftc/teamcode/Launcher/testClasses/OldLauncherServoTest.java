package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;
import org.firstinspires.ftc.teamcode.Launcher.LauncherServo;

@TeleOp(group = "Launcher", name = "Pitch")
public class LauncherServoTest extends LinearOpMode {

    public LauncherServo launcherServo;

    @Override
    public void runOpMode() {

        HWMap hwMap = new HWMap(hardwareMap);
        launcherServo = new LauncherServo(telemetry, hwMap);

        waitForStart();
        while (opModeIsActive()) {
            launcherServo.updatePID();
            telemetry.update();
        }
    }
}
