package org.firstinspires.ftc.teamcode.team10515.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Program 1", group="XtremeV")
public class basicauto extends UGBase {

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;
    private double headingResetValue;

    private static double speed_value = 0.3;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //  initVuforia();

        telemetry.addData("Status: ", "Ready to run");    //
        telemetry.update();
        //String position = getSkyStonePosition();

        waitForStart();
        move_forward(0.5,45);
        move_sideways_rampup(0,0.5,40,16,false);
        //shoot
        move_right(0.5,20);
        move_forward(0.5,20);

    }
}
