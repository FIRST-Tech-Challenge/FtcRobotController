package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Near", group="Linear Opmode")
public class AutoRedNear extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = true;
        super.runOpMode();
        df.TurnToHeading(TURN_SPEED, -90);
        df.DriveStraight(DRIVE_SPEED, 40, -90);
    }
}
