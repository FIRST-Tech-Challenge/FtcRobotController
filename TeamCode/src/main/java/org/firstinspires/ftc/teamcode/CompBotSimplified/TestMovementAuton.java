package org.firstinspires.ftc.teamcode.CompBotSimplified;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

// Start blue storage side

@Autonomous
@Disabled

public class TestMovementAuton extends LinearOpMode {
    public static final double dPower = 0.75;

    CompBotHWSimplified robot = new CompBotHWSimplified();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        robot.init(hardwareMap,true);
        boolean[] pos = {false,false,false};
        while(!isStarted()) {
             pos = robot.p.getPositions();
        }

        // line up with drop
        robot.assistedEncoderDrive(-24,0,-dPower,0);
        robot.assistedEncoderDrive(0,18,0,dPower);

        // lift and drop
        if (Arrays.equals(pos, new boolean[]{true, true, false})) {// left
            // lift up
            // intake open
            // lift down
        } else if (Arrays.equals(pos, new boolean[]{true, false, true})) {// middle
        } else {// right
        }

        // Go back
        robot.assistedEncoderDrive(0,-16,0,-dPower);

        // Rotate to correct orientation
        robot.gyroTurn(180,dPower);
        robot.assistedEncoderDrive(0,4,0,0.15); // bang against wall

        // Strafe over to carousel
        robot.assistedEncoderDrive(-36,0,-dPower,0);
        robot.assistedEncoderDrive(6,0,-0.15,0);

        // Turn on motor, wait, then turn off

        // Strafe to warehouse
        robot.assistedEncoderDrive(200,0,1,0);

        robot.m.stop();


    }
}