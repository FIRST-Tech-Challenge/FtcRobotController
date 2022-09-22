package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateDelayTenSeconds implements EbotsAutonState{

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Class Attributes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Instance Attributes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    StopWatch stopWatch;
    EbotsAutonOpMode autonOpMode;
    long stateTimeLimit;
    int loopCount = 0;
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Constructors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateDelayTenSeconds(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        stopWatch = new StopWatch();
        stateTimeLimit = 10000;
        this.autonOpMode = autonOpMode;

    }
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Getters & Setters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Class Methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Instance Methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public boolean shouldExit() {
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;

        return stateTimedOut | !autonOpMode.opModeIsActive();
}

    @Override
    public void performStateActions() {
        autonOpMode.telemetry.addLine(stopWatch.toString());
        boolean debugOn = loopCount % 100 == 0;
        if (debugOn){
            double currentHeading = EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true);
            Log.d("EBOTS", "Current Heading:" + String.format("%.2f", currentHeading));
        }
        loopCount ++;
    }

    @Override
    public void performTransitionalActions() {
        autonOpMode.telemetry.addLine("Exiting state StateDelayFiveSeconds");
    }
}
