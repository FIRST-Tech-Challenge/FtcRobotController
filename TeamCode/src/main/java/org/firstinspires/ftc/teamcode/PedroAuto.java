package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.PedroPathingSpecimenBot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;

@Autonomous(name = "Pedro Auto", group = "Auto")
public class PedroAuto extends LinearOpMode {
    protected PedroPathingSpecimenBot robot = new PedroPathingSpecimenBot(this);
    private PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.isAuto = true;
        robot.init(hardwareMap);
        robot.onLoop(10, "pedro auto");
        waitForStart();
        while (opModeIsActive()) {


            telemetry.addData("path state", robot.getPathState());

            telemetry.update();
        }
    }

}
