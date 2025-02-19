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

        waitForStart();
        while (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.onLoop(10, "pedro auto");
            telemetry.addData("path state", robot.getPathState());

            telemetry.update();
        }
    }

}
