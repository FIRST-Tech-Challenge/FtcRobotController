package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private Hardware hardware;

    // Red or blue team
    private TeamColor teamColor;
    // Far or near
    private TeamSide teamSide;

    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        hardware = new Hardware(this);

        telemetry.addData("Initial position:", teamColor.name() + ' ' + teamSide.name());

        // Wait until the player press the start button
        waitForStart();
    }
}