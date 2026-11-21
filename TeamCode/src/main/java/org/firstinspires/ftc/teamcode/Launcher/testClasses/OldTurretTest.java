package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;
import org.firstinspires.ftc.teamcode.Launcher.Turret;

@TeleOp(group = "Launcher", name = "Turret")
public class TurretTest extends LinearOpMode {

    public Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {

        HWMap hwMap = new HWMap(hardwareMap);
        turret = new Turret(hwMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            turret.updatePID();
            telemetry.update();
        }
    }
}
