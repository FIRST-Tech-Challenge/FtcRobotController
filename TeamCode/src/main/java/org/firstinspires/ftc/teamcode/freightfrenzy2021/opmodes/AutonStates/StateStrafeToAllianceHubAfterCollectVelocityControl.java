package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;
import org.firstinspires.ftc.teamcode.ebotsenums.RobotSize;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Arm;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateStrafeToAllianceHubAfterCollectVelocityControl extends EbotsAutonStateVelConBase{

    private final double HUB_DISTANCE_FROM_WALL = 48.0;
    private final double ROBOT_HALF_WIDTH = RobotSize.ySize.getSizeValue();
    private final double BUCKET_CENTER_OFFSET =1.0;
    private final static double OVERTRAVEL_INCHES = 6.0;


    public StateStrafeToAllianceHubAfterCollectVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define

        int allianceSign = (AllianceSingleton.isBlue()) ? 1 : -1;

        // because the bucket position is asymmetrical, the drive distance from the wall must
        // be adjusted based on alliance.  If red, subtract from travel distance.  add if blue

        double pushOffDistance = StatePushOffWithVelocityControl.getTravelDistance();
        Log.d(logTag, "Acquired travelDistance from StatePushOffWithVelocityControl: " +
                String.format(twoDec, pushOffDistance));

        travelDistance = HUB_DISTANCE_FROM_WALL - ROBOT_HALF_WIDTH +
                (BUCKET_CENTER_OFFSET * allianceSign) + OVERTRAVEL_INCHES;

        travelDirectionDeg = AllianceSingleton.isBlue() ? -90.0 : 90.0;
        targetHeadingDeg = 0.0;

        initAutonState();
        setDriveTarget();
        moveArmToTargetLevel();

        Log.d(logTag, "Constructor complete");
    }

    @Override
    public boolean shouldExit() {
        return super.shouldExit();
    }

    @Override
    public void performStateActions() {
        super.performStateActions();
    }

    @Override
    public void performTransitionalActions() {
        super.performTransitionalActions();
    }

    private void moveArmToTargetLevel() {
//        BarCodePosition barCodePosition = autonOpMode.getBarCodePosition();
//        Arm.Level targetLevel = Arm.Level.ONE;
//        if (barCodePosition == BarCodePosition.MIDDLE) {
//            targetLevel = Arm.Level.TWO;
//        } else if (barCodePosition == BarCodePosition.RIGHT) {
//            targetLevel = Arm.Level.THREE;
//        }
        Arm.Level targetLevel = Arm.Level.THREE;
        Arm arm = Arm.getInstance(autonOpMode);
        arm.moveToLevel(targetLevel);
    }
}
