package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Far", group="Linear Opmode")
public class AutoRedFar extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = true;
        super.runOpMode();
        df.turnToHeading(DRIVE_SPEED, 0);
        df.driveStraight(DRIVE_SPEED, 18, 0);
        df.turnToHeading(DRIVE_SPEED, -90);
        df.driveStraight(1, 60, -90);
        df.turnToHeading(DRIVE_SPEED, -30);
        df.driveStraight(1, 40, -30);
    }
}
