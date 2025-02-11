package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.PedroPathingSpecimenBot;

@Autonomous(name = "Pedro Auto", group = "Auto")
public class PedroAuto extends LinearOpMode {
    protected PedroPathingSpecimenBot robot = new PedroPathingSpecimenBot(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.onLoop(10, "pedro auto");


            telemetry.update();
        }
    }

}
