package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Far", group="Linear Opmode")
public class AutoRedFar extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = true;
        super.runOpMode();
        df.TurnToHeading(DRIVE_SPEED, 0);
        df.DriveStraight(DRIVE_SPEED, 18, 0, false);
        df.TurnToHeading(DRIVE_SPEED, -90);
        df.DriveStraight(1, 60, -90, false);
        df.TurnToHeading(DRIVE_SPEED, -30);
        df.DriveStraight(1, 40, -30, false);
    }
}
