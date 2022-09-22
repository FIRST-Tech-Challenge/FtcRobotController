/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes;

import android.util.Log;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.Pose2020;
import org.firstinspires.ftc.teamcode.ultimategoal2020.fieldobjects2020.StartLine;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.AbstractAutonState;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.AutonState;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.AutonStateFactory;
import org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates.StateInitialize;


/**
 * This is an autonomous routine using a state machine
 * It is derived off linear opmode and traverses states which implement
 * AutonState interface
 *
 * Note that during state PreMatchSetup that encoder values can be read for manual calibration
 */

@Autonomous(name="AutonEbotsV1", group="Competition")
@Disabled
public class AutonEbotsV1 extends LinearOpMode {

    //initializing and declaring class attributes
    private AutonParameters autonParameters;
    private EbotsRobot2020 robot;

    private AutonStateFactory autonStateFactory = new AutonStateFactory();
    private AutonState autonState;
    private AutonRoutine autonRoutine = new AutonRoutine(PresetAutonRoutine.STANDARD);

//    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    private StartLine.LinePosition startLinePosition = StartLine.LinePosition.INNER;

    final boolean debugOn = true;
    final String logTag = "EBOTS";

    public StartLine.LinePosition getStartLinePosition() {
        return startLinePosition;
    }

    public void setStartLinePosition(StartLine.LinePosition startLinePosition) {
        this.startLinePosition = startLinePosition;
    }

    // Getters


//    public FtcDashboard getDashboard() {
//        return dashboard;
//    }

    public Telemetry getDashboardTelemetry(){
        return dashboardTelemetry;
    }

    @Override
    public void runOpMode(){
        if(debugOn) Log.d(logTag, "Entering runOpMode for AutonEbotsV1");
        initializeRobot();

//        //Configure FtcDashboard telemetry
//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
//        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        Class<? extends AbstractAutonState> firstStateClass = autonRoutine.getNextAutonStateClass();
        Class<? extends AbstractAutonState> nextStateClass = autonRoutine.getNextAutonStateClass();
        autonState = autonStateFactory.getAutonState(firstStateClass,this, robot, nextStateClass);

        // Create two state machines.  The first one is prior to pushing "Start"
        // The second is after pushing start
        preMatchStateMachine();

        // NOTE:  wiatForStart() must be called or the compiler will flag an error
        waitForStart();

        // This second state machine is intended for executing the auton routine such as:
        //  INITIALIZE, ALL_AUTON_ACTIONS
        matchPlayStateMachine();
    }


    private void preMatchStateMachine() {
        // This first state machine is intended for states prior to starting routine such as:
        //  CONFIGURE_AUTON_ROUTINE, PREMATCH_SETUP, DETECT_STARTER_STACK
        //  NOTE:  All states within consideration should check for isStarted in exit conditions

        while (!isStarted() && autonState.getCurrentAutonState() != StateInitialize.class){
            executeStateMachine();  //
        }

        // Perform a log dump if reached this point and not in initialize
        if(autonState.getCurrentAutonState() != StateInitialize.class){
            Log.d(logTag, "AutonEbotsV1::runOpMode First state machine exited and not in INITIALIZE but: " + autonState.getCurrentAutonState().getSimpleName());
        }
    }


    private void matchPlayStateMachine() {
        while (opModeIsActive()){
            executeStateMachine();
        }
    }


    private void executeStateMachine() {
        /**
         * Executes a state machine:
         * a) Checks for exit conditions
         * b) If met:       Performs transitional actions (state specific and general)
         * c) If not met:   Performs state actions
         */
        if (autonState.areExitConditionsMet()) {
            // Perform state-specific transition actions
            autonState.performStateSpecificTransitionActions();
            // Perform standard transition actions, including setting the next autonState
            performStandardStateTransitionActions();
        } else {
            autonState.performStateActions();
        }
    }


    public void performStandardStateTransitionActions(){
        telemetry.clearAll();
        robot.getEbotsMotionController().resetLoopVariables();
        this.incrementState();
    }


    private void incrementState(){
        //Set the next AutonState
        Class<? extends AbstractAutonState> targetStateClass = autonState.getNextAutonState();
        Class<? extends AbstractAutonState> nextStateClass = autonRoutine.getNextAutonStateClass();
        if(debugOn) Log.d(logTag, "Attempting to create state: " + targetStateClass.getSimpleName() +
                " with nextState: " + nextStateClass.getSimpleName());
        autonState = autonStateFactory.getAutonState(targetStateClass,this, robot, nextStateClass);
    }


    private void initializeRobot() {
        if(debugOn) Log.d(logTag, "Entering AutonEbotsV1::initializeRobot...");
        Alliance tempAlliance = Alliance.RED;
        Pose2020 startingPose2020 = new Pose2020(startLinePosition, tempAlliance);

        autonParameters = AutonParameters.COMPETITION;

        //  this constructor also builds the drive wheel motors and manip devices and encoders
        robot = new EbotsRobot2020(startingPose2020, tempAlliance, autonParameters, hardwareMap);


        telemetry.addLine(robot.getActualPose().toString());
        telemetry.addLine("Initialize Complete!");
        telemetry.update();
    }
}
