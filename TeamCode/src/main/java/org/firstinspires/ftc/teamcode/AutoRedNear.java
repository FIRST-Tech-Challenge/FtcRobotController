package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Near", group="Linear Opmode")
public class AutoRedNear extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = true;
        super.runOpMode();
        df.TurnToHeading(TURN_SPEED, 0);
        df.DriveStraight(DRIVE_SPEED, -10, 0, false);
        df.DriveStraight(DRIVE_SPEED, 48, 0, true);
        df.DriveStraight(DRIVE_SPEED, 13, 0, false);
        sf.PutPixelInBackBoard();
    }
}
