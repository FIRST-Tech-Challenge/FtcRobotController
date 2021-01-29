package org.firstinspires.ftc.teamcode.autonomous.sequences;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.action.DetectRingsAction;
import org.firstinspires.ftc.teamcode.action.ExecuteSequenceAction;
import org.firstinspires.ftc.teamcode.action.IfActionResult;
import org.firstinspires.ftc.teamcode.action.LocalizerMoveAction;
import org.firstinspires.ftc.teamcode.action.MoveAndOrientAction;
import org.firstinspires.ftc.teamcode.action.WaitForeverAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class UltimateGoalSequence extends ActionSequence {

    public static RobotTransform[] A_Near_Center_Transforms = {
            new RobotTransform(DistanceUnit.INCH, -24, -12, 135),
            new RobotTransform(DistanceUnit.INCH, 12, -36, 90)
    };

    public static RobotTransform[] A_Near_Wall_Transforms = {
            new RobotTransform(DistanceUnit.INCH, -24, -60, 90),
            new RobotTransform(DistanceUnit.INCH, -8, -60, 0)

    };

    public static RobotTransform[] B_Near_Center_Transforms = {
            new RobotTransform(DistanceUnit.INCH, -24, -12, 135),
            new RobotTransform(DistanceUnit.INCH, 36, -12, 90)
    };

    public static RobotTransform[] B_Near_Wall_Transforms = {
            new RobotTransform(DistanceUnit.INCH, -24, -60, 90),
            new RobotTransform(DistanceUnit.INCH, 36, -60, 90),
            new RobotTransform(DistanceUnit.INCH, 36, -60, -90)
    };

    public static RobotTransform[] C_Near_Center_Transforms = {
            new RobotTransform(DistanceUnit.INCH, -24, -12, 135),
            new RobotTransform(DistanceUnit.INCH, 48, -36, 90),
            new RobotTransform(DistanceUnit.INCH, 60l, -36, 90),
    };

    public static RobotTransform[] C_Near_Wall_Transforms = {
            new RobotTransform(DistanceUnit.INCH, -24, -60, 90),
            new RobotTransform(DistanceUnit.INCH, 36, -60, 90),
            new RobotTransform(DistanceUnit.INCH, 36, -60, 0)
    };

    public static RobotTransform[] RING_DETECTION_TRANSFORMS_NEAR_CENTER = {
            new RobotTransform(DistanceUnit.INCH, -48,-52,90),
            new RobotTransform(DistanceUnit.INCH, -48,-48,120)
    };

    public static RobotTransform[] RING_DETECTION_TRANSFORMS_NEAR_WALL = {
            new RobotTransform(DistanceUnit.INCH, -48,-52,90),
            new RobotTransform(DistanceUnit.INCH, -48,-48,120)
    };

    static final double ROBOT_SPEED = 0.75;
    static final double ROBOT_PRECISE_SPEED = 0.42f;
    static final LocalizerMoveAction.FollowPathMethod FOLLOW_PATH_METHOD = LocalizerMoveAction.FollowPathMethod.LINEAR;
    static final RobotTransform SHOOTING_POSITION_NEAR_CENTER = new RobotTransform(DistanceUnit.INCH, 0, -36, 85);
    static final RobotTransform SHOOTING_POSITION_NEAR_WALL = new RobotTransform(DistanceUnit.INCH, 0, -36, 85);
    static final RobotTransform PARKING_POSITION_NEAR_CENTER = new RobotTransform(DistanceUnit.INCH, 12, -12, 90);
    static final RobotTransform PARKING_POSITION_NEAR_WALL = new RobotTransform(DistanceUnit.INCH, 12, -36, 90);

    public UltimateGoalSequence(RobotHardware.Team team, UltimateGoalHardware.UltimateGoalStartingPosition startingPosition) {
        // Setup transforms
        RobotTransform[] shootingPosition;
        RobotTransform[] parkingPosition;
        RobotTransform[] ringDetectionPosition;

        // The transforms to use for different ring positions
        // 0 - No rings
        // 1 - Single ring
        // 2 - Quad rings
        RobotTransform[][] ringTransforms = new RobotTransform[3][];

        if (team == RobotHardware.Team.RED && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.LEFT) {
            // RED LEFT
            ringTransforms[0] = A_Near_Center_Transforms;
            ringTransforms[1] = B_Near_Center_Transforms;
            ringTransforms[2] = C_Near_Center_Transforms;
            shootingPosition = new RobotTransform[] { SHOOTING_POSITION_NEAR_CENTER };
            parkingPosition = new RobotTransform[] { PARKING_POSITION_NEAR_CENTER };
            ringDetectionPosition = RING_DETECTION_TRANSFORMS_NEAR_CENTER;
        } else if (team == RobotHardware.Team.RED && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.RIGHT) {
            // RED RIGHT
            ringTransforms[0] = A_Near_Wall_Transforms;
            ringTransforms[1] = B_Near_Wall_Transforms;
            ringTransforms[2] = C_Near_Wall_Transforms;
            shootingPosition = new RobotTransform[] { SHOOTING_POSITION_NEAR_WALL };
            parkingPosition = new RobotTransform[] { PARKING_POSITION_NEAR_WALL };
            ringDetectionPosition = RING_DETECTION_TRANSFORMS_NEAR_WALL;
        } else if (team == RobotHardware.Team.BLUE && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.LEFT) {
            // BLUE LEFT
            ringTransforms[0] = Localizer.mirrorTransformsOverTeamLine(A_Near_Wall_Transforms);
            ringTransforms[1] = Localizer.mirrorTransformsOverTeamLine(B_Near_Wall_Transforms);
            ringTransforms[2] = Localizer.mirrorTransformsOverTeamLine(C_Near_Wall_Transforms);
            shootingPosition = new RobotTransform[] { SHOOTING_POSITION_NEAR_WALL };
            parkingPosition = new RobotTransform[] { PARKING_POSITION_NEAR_WALL};
            ringDetectionPosition = Localizer.mirrorTransformsOverTeamLine(RING_DETECTION_TRANSFORMS_NEAR_WALL);
            shootingPosition = Localizer.mirrorTransformsOverTeamLine(shootingPosition);
            parkingPosition = Localizer.mirrorTransformsOverTeamLine(parkingPosition);
            ringDetectionPosition = Localizer.mirrorTransformsOverTeamLine(ringDetectionPosition);
        } else {
            // BLUE RIGHT
            ringTransforms[0] = Localizer.mirrorTransformsOverTeamLine(A_Near_Center_Transforms);
            ringTransforms[1] = Localizer.mirrorTransformsOverTeamLine(B_Near_Center_Transforms);
            ringTransforms[2] = Localizer.mirrorTransformsOverTeamLine(C_Near_Center_Transforms);
            shootingPosition = new RobotTransform[] { SHOOTING_POSITION_NEAR_CENTER };
            parkingPosition = new RobotTransform[] { PARKING_POSITION_NEAR_CENTER };
            ringDetectionPosition = Localizer.mirrorTransformsOverTeamLine(RING_DETECTION_TRANSFORMS_NEAR_CENTER);
            shootingPosition = Localizer.mirrorTransformsOverTeamLine(shootingPosition);
            parkingPosition = Localizer.mirrorTransformsOverTeamLine(parkingPosition);
            ringDetectionPosition = Localizer.mirrorTransformsOverTeamLine(ringDetectionPosition);
        }

        // Move to ring detection position
        addAction(new LocalizerMoveAction(ringDetectionPosition, ROBOT_SPEED, ROBOT_PRECISE_SPEED, LocalizerMoveAction.FollowPathMethod.LINEAR));

        // Detect rings
        DetectRingsAction ringDetectionAction = new DetectRingsAction(2000);
        addAction(ringDetectionAction);

        // Go to the correct wobble goal zone depending on what the camera detected
        // A - No rings
        addAction(new IfActionResult(
                ringDetectionAction,
                DetectRingsAction.DetectRingsResult.NONE,
                new LocalizerMoveAction(ringTransforms[0], ROBOT_SPEED, ROBOT_PRECISE_SPEED, FOLLOW_PATH_METHOD),
                null));

        // B - Single ring
        addAction(new IfActionResult(
                ringDetectionAction,
                DetectRingsAction.DetectRingsResult.SINGLE,
                new LocalizerMoveAction(ringTransforms[1], ROBOT_SPEED, ROBOT_PRECISE_SPEED, FOLLOW_PATH_METHOD),
                null));

        // C - Quad rings
        addAction(new IfActionResult(
                ringDetectionAction,
                DetectRingsAction.DetectRingsResult.QUAD,
                new LocalizerMoveAction(ringTransforms[2], ROBOT_SPEED, ROBOT_PRECISE_SPEED, FOLLOW_PATH_METHOD),
                null));

        // Release the wobble goal
        addAction(new ExecuteSequenceAction(new ReleaseWobbleGoalSequence()));

        // Move to shooting position
        addAction(new LocalizerMoveAction(shootingPosition, ROBOT_SPEED, ROBOT_PRECISE_SPEED, FOLLOW_PATH_METHOD));

        // Shoot three rings into high goal
        addAction(new ExecuteSequenceAction(new ShootActionSequence(3)));

        // Park on center line
        addAction(new LocalizerMoveAction(parkingPosition, ROBOT_SPEED, ROBOT_PRECISE_SPEED, FOLLOW_PATH_METHOD));

        addAction(new WaitForeverAction());
    }
}
