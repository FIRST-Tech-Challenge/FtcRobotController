package org.firstinspires.ftc.teamcode.autonomous.sequences;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.action.DetectRingsAction;
import org.firstinspires.ftc.teamcode.action.IfActionResult;
import org.firstinspires.ftc.teamcode.action.MoveAlongPathAction;
import org.firstinspires.ftc.teamcode.hardware.UltimateGoalHardware;
import org.firstinspires.ftc.teamcode.playmaker.ActionSequence;
import org.firstinspires.ftc.teamcode.playmaker.Localizer;
import org.firstinspires.ftc.teamcode.playmaker.Localizer.RobotTransform;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class UltimateGoalSequence extends ActionSequence {

    public static RobotTransform[] A_Left_Transforms = {
            new RobotTransform(new Position(DistanceUnit.INCH, 0,0,0, 0), 0)
    };

    public static RobotTransform[] A_Right_Transforms = {
            new RobotTransform(new Position(DistanceUnit.INCH, -12,-60,0, 0), 0)
    };

    public static RobotTransform[] B_Left_Transforms = {
            new RobotTransform(new Position(DistanceUnit.INCH, 0,0,0, 0), 0)
    };

    public static RobotTransform[] B_Right_Transforms = {
            new RobotTransform(new Position(DistanceUnit.INCH, 36,-60,0, 0), 0)
    };

    public static RobotTransform[] C_Left_Transforms = {
            new RobotTransform(new Position(DistanceUnit.INCH, 0,0,0, 0), 0)
    };

    public static RobotTransform[] C_Right_Transforms = {
            new RobotTransform(new Position(DistanceUnit.INCH, 36,-60,0, 0), 0)
    };


    static final double ROBOT_SPEED = 0.75;
    static final MoveAlongPathAction.FollowPathMethod FOLLOW_PATH_METHOD = MoveAlongPathAction.FollowPathMethod.LINEAR;

    public UltimateGoalSequence(RobotHardware.Team team, UltimateGoalHardware.UltimateGoalStartingPosition startingPosition) {

        // Detect rings
        DetectRingsAction ringDetectionAction = new DetectRingsAction(2000);
        addAction(ringDetectionAction);

        RobotTransform[][] ringTransforms = new RobotTransform[3][];
        if (team == RobotHardware.Team.RED && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.LEFT) {
            ringTransforms[0] = A_Left_Transforms;
            ringTransforms[1] = B_Left_Transforms;
            ringTransforms[2] = C_Right_Transforms;
        } else if (team == RobotHardware.Team.RED && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.RIGHT) {
            ringTransforms[0] = A_Right_Transforms;
            ringTransforms[1] = B_Right_Transforms;
            ringTransforms[2] = C_Right_Transforms;
        } else if (team == RobotHardware.Team.BLUE && startingPosition == UltimateGoalHardware.UltimateGoalStartingPosition.LEFT) {
            // We use the right transforms for the left starting position because they start closer to the wall
            ringTransforms[0] = Localizer.mirrorTransformsOverTeamLine(A_Right_Transforms);
            ringTransforms[1] = Localizer.mirrorTransformsOverTeamLine(B_Right_Transforms);
            ringTransforms[2] = Localizer.mirrorTransformsOverTeamLine(C_Right_Transforms);
        } else {
            ringTransforms[0] = Localizer.mirrorTransformsOverTeamLine(A_Left_Transforms);
            ringTransforms[1] = Localizer.mirrorTransformsOverTeamLine(B_Left_Transforms);
            ringTransforms[2] = Localizer.mirrorTransformsOverTeamLine(C_Left_Transforms);
         }

        addAction(new IfActionResult(
                ringDetectionAction,
                DetectRingsAction.DetectRingsResult.NONE,
                new MoveAlongPathAction(ringTransforms[0], ROBOT_SPEED, FOLLOW_PATH_METHOD),
                null));

        addAction(new IfActionResult(
                ringDetectionAction,
                DetectRingsAction.DetectRingsResult.SINGLE,
                new MoveAlongPathAction(ringTransforms[1], ROBOT_SPEED, FOLLOW_PATH_METHOD),
                null));

        addAction(new IfActionResult(
                ringDetectionAction,
                DetectRingsAction.DetectRingsResult.QUAD,
                new MoveAlongPathAction(ringTransforms[2], ROBOT_SPEED, FOLLOW_PATH_METHOD),
                null));

        //addAction(new ReleaseWobbleGoalAction());

    }
}
