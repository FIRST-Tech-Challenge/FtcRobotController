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
            df.DriveStraight(1, 38, 0, true);
        } else if (circleDetection.GetBallPosition() == CircleDetection.BallPosition.CENTER) {
            df.DriveStraight(1, 48, 0, true);
            df.DriveStraight(DRIVE_SPEED, 3, 0, false);
        } else if (circleDetection.GetBallPosition() == CircleDetection.BallPosition.LEFT) {
            df.DriveStraight(1, 58, 0, true);
            df.DriveStraight(DRIVE_SPEED, 7, 0, false);
            df.DriveStraight(DRIVE_SPEED, 5, 0, false);
        }
        sf.PutPixelInBackBoard();
        df.DriveStraight(DRIVE_SPEED, -20, 0, false);
    }
}
