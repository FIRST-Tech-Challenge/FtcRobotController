package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

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

        // teamColor = (hardware.getColorSwitch().getState()) ? TeamColor.RED : TeamColor.BLUE;
        // teamSide = (hardware.getSideSwitch().getState()) ? TeamSide.FAR : TeamSide.NEAR;

        // telemetry.addData("Initial position:", teamColor.name() + ' ' + teamSide.name());

        // Wait until the player press the start button
        waitForStart();

        CRServo intakeServo = hardware.getArm().getIntakeServo();
        telemetry.addData("intakeServo: ", intakeServo);

        while (opModeIsActive()) {
            hardware.getArm().startIntake();
        }

        sleep(60000);
    }
}