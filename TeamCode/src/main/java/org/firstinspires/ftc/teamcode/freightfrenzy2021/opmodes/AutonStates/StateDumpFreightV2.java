package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;
import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Arm;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;


public class StateDumpFreightV2 extends EbotsAutonStateVelConBase{
    private StopWatch stopWatchDump;
    private Bucket bucket;
    private Arm arm;
    private final long dumpTime;
    private boolean dumpStarted = false;

    public StateDumpFreightV2(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);

        bucket = Bucket.getInstance(autonOpMode);
        arm = Arm.getInstance (autonOpMode);
        // This is commented out because limit switch is not working.
        //arm.zeroArmHeight();
        stopWatchDump = new StopWatch();
        stateTimeLimit = 2500L;
        dumpTime = 1250L;
    }

    @Override
    public boolean shouldExit() {
        boolean dumpComplete = dumpStarted && stopWatchDump.getElapsedTimeMillis() > dumpTime;
        boolean stateTimedOut = stopWatchState.getElapsedTimeMillis() > stateTimeLimit;

        if(dumpComplete) Log.d(logTag, "State exited because dump complete");
        if(stateTimedOut) Log.d(logTag, "State exited because timed out");
        if(!autonOpMode.opModeIsActive()) Log.d(logTag, "State exited because opmode inactivated");
        return stateTimedOut | dumpComplete | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        if (!dumpStarted && arm.isAtTargetLevel()){
            bucket.setState(BucketState.DUMP);
            dumpStarted = true;
            arm.getArmState();
            stopWatchDump.reset();
        } else if (dumpStarted){
            // once dumping, must continue to set state to get bucket waggle
            bucket.setState(BucketState.DUMP);
        }
    }

    @Override
    public void performTransitionalActions() {
        Log.d(logTag, stopWatchState.toString());
        telemetry.addLine("Transitional Actions of " + this.getClass().getSimpleName());
        arm.moveToLevel(Arm.Level.ONE);
        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
    }
}
