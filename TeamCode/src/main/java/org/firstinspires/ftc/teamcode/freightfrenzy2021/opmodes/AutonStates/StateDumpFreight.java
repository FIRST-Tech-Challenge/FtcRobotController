package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;
import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Arm;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;


public class StateDumpFreight implements EbotsAutonState{
    private EbotsAutonOpMode autonOpMode;
    private StopWatch stopWatchState;
    private StopWatch stopWatchDump;
    private Bucket bucket;
    private Arm arm;
    private final long stateTimeLimit;
    private final long dumpTime;
    private Arm.Level targetLevel;
    private boolean targetLevelAchieved = false;
    private String logTag = "EBOTS";

    public StateDumpFreight(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        //stopWatchDump re-instantiation necessary??
        bucket = Bucket.getInstance(autonOpMode);
        arm = Arm.getInstance (autonOpMode);
        // This is commented out because limit switch is not working.
        //arm.zeroArmHeight();
        stopWatchState = new StopWatch();
        stopWatchDump = new StopWatch();
        stateTimeLimit = 5000L;
        dumpTime = 1250L;
        BarCodePosition barCodePosition = autonOpMode.getBarCodePosition();

        //BarCodePosition barCodePosition = BarCodePosition.RIGHT;


        if(bucket.getBucketState() != BucketState.TRAVEL){
            StopWatch stopWatch = new StopWatch();
            bucket.setState(BucketState.TRAVEL);
            while(autonOpMode.opModeIsActive() && stopWatch.getElapsedTimeMillis() < 350){
                // wait for the bucket to move to travel state
            }
        }

        targetLevel = Arm.Level.ONE;
        if (barCodePosition == BarCodePosition.MIDDLE){
            targetLevel = Arm.Level.TWO;
        } else if (barCodePosition == BarCodePosition.RIGHT){
            targetLevel = Arm.Level.THREE;
        }
        if (targetLevel == Arm.Level.ONE){
            targetLevelAchieved = true;
            stopWatchDump.reset();
            bucket.setState(BucketState.DUMP);
        } else {
            arm.moveToLevelAuton(targetLevel);
        }
    }

    @Override
    public boolean shouldExit() {
        boolean dumpTimeCompleted = stopWatchDump.getElapsedTimeMillis() > dumpTime;
        boolean dumpComplete = targetLevelAchieved && dumpTimeCompleted;
        boolean stateTimedOut = stopWatchState.getElapsedTimeMillis() > stateTimeLimit;
        if(dumpComplete) Log.d(logTag, "State exited because dump complete");
        if(stateTimedOut) Log.d(logTag, "State exited because timed out");
        if(!autonOpMode.opModeIsActive()) Log.d(logTag, "State exited because opmode inactivated");
        return stateTimedOut | dumpComplete | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        if (arm.isAtTargetLevel() && !targetLevelAchieved){
            stopWatchDump.reset();
            targetLevelAchieved = true;
            bucket.setState(BucketState.DUMP);
        } else if (targetLevelAchieved){
            // once dumping, must continue to set state to get bucket waggle
            bucket.setState(BucketState.DUMP);
        }
    }

    @Override
    public void performTransitionalActions() {
        bucket.setState(BucketState.TRAVEL);
        arm.moveToLevelAuton(Arm.Level.ONE);
    }
}
