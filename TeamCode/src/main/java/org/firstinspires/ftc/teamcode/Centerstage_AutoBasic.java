package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Centerstage Auto Basic", group = "Concept")
public class Centerstage_AutoBasic extends LinearOpMode {
    @Override
    public void runOpMode() {
        Gobbler gobbler = new Gobbler(hardwareMap);

        waitForStart();

        gobbler.driveTrain.moveForward(5, 0.5);
        gobbler.driveTrain.Wait(0.25);
        gobbler.driveTrain.strafe(-36, 0.5);
    }
}
