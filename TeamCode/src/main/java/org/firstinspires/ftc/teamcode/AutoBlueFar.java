package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Blue - Far", group="Linear Opmode")
public class AutoBlueFar extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = false;
        super.runOpMode();
        df.TurnToHeading(TURN_SPEED, 0);
        df.DriveStraight(DRIVE_SPEED, 18, 0);
        df.TurnToHeading(TURN_SPEED, 90);
        df.DriveStraight(1, 60, 90);
        df.TurnToHeading(TURN_SPEED, 30);
        df.DriveStraight(1, 40, 30);
    }
}
