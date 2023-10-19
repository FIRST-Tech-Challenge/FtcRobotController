package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto - Red - Near", group="Linear Opmode")
public class AutoRedNear extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = true;
        super.runOpMode();
        if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.RIGHT) {
            df.DriveStraight(DRIVE_SPEED, 42, 0, true);
        } else if (circleDetection.GetBallPosition() == CircleDetection.BallPosition.CENTER) {
            df.DriveStraight(DRIVE_SPEED, 42, 0, true);
            df.DriveStraight(DRIVE_SPEED, 7, 0, false);
        } else if (circleDetection.GetBallPosition() == CircleDetection.BallPosition.LEFT) {
            df.DriveStraight(DRIVE_SPEED, 42, 0, true);
            df.DriveStraight(DRIVE_SPEED, 10, 0, false);
            df.DriveStraight(DRIVE_SPEED, -10, 0, false);
        }
        sf.PutPixelInBackBoard();
        df.DriveStraight(DRIVE_SPEED, -5, 0, true);
        df.DriveStraight(DRIVE_SPEED, -35, 0, false);
        df.DriveStraight(DRIVE_SPEED, 10, 0, true);
    }
}
