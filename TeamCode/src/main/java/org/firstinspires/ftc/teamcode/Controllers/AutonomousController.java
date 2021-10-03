package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.SubsystemTemplate;

/**
 * Defenition of the AutoControl Class <BR>
 *
 * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
 * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
 * 
 */

public class AutonomousController {

    //Create gamepad object reference to connect to gamepad1
    public DriveTrain driveTrain;
    public SubsystemTemplate subsystemTemplate;
    //TODO: Replace name of Subsystem1 and Declare more subsystems

    public Pose2d startPose = GameField.BLUE_STARTPOS_1;

    // TODO: Declare autonomous option logic based on key pad selection
    /* Example
    public boolean launchHighGoalOrPowerShot = false;
    public boolean dropFirstWobbleGoal = false;
    public boolean pickRingFromTargetMarker = false;
    public boolean launchRingsPickedFromTargetMarkerToHighGoal = false;
    public boolean pickAndDropSecondWobbleGoal = false;
         */

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * TODO: Add more subsystems in declaration
     */
    public AutonomousController(DriveTrain driveTrain,
                                SubsystemTemplate subsystemTemplate) {
        this.driveTrain = driveTrain;
        this.subsystemTemplate = subsystemTemplate;
        //TODO: Add more subsystems
    }

    /**
     * Main control function that calls the runs of each of the subsystems in sequence
     */
    public void runAutoControl(){
        int counter = 0;
        while (counter < 5) {
            runSubsystem1Control();
            //TODO: Add runSubsystemControl functions for each subsystem
            counter++;
        }
    }



    // Define and delcare autonomous states
    // Example
    enum AUTO_SUBSYSTEM1_STATE{
        START,
        STOP,
        //TODO:Update Subsystem states as appropriate
    }
    AUTO_SUBSYSTEM1_STATE autoSubsystem1State = AUTO_SUBSYSTEM1_STATE.STOP;

    //TODO: Add states for each Subsystem

    /**
     * run Intake Control State machine response
     * Also deactivate Launch readiness when Intake is started
     */
    public void runSubsystem1Control(){

        if (autoSubsystem1State == AUTO_SUBSYSTEM1_STATE.START){
            //TODO: Add state setting code for Subsystem1
            /* Set state for subsystem - Example
            acHzLaunchSubControllerUltimateGoal.activateLaunchReadinessState = false;
            acHzLaunchSubControllerUltimateGoal.deactivateLaunchReadinessState = true;
            acHzMagazineUltimateGoal.moveMagazineTo = HzMagazineUltimateGoal.MOVE_MAGAZINE_TO.COLLECT;
             */
        }

        if (autoSubsystem1State == AUTO_SUBSYSTEM1_STATE.STOP){
            //TODO: Add state setting code for Subsystem1
             /* Set state for subsystem - Example
            acHzIntakeUltimateGoal.intakeButtonState = HzIntakeUltimateGoal.INTAKE_BUTTON_STATE.OFF;
              */
        }
    }

    //TODO: Add more run Subsystem Control functions

}