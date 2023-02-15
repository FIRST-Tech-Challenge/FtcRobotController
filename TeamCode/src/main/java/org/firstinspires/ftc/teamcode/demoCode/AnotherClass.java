package org.firstinspires.ftc.teamcode.demoCode;

import java.util.Scanner;

public class AnotherClass {

    public static Scanner userInput = new Scanner(System.in);
    public enum operatorStates {
        IDLE,
        LINE_UP,
        MOVE_SUBSYSTEMS_PICKUP,
        INTAKE,
        LINE_UP_APRILTAG,
        DETECT_DELIVERY,
        MOVE_SUBSYSTEMS_DELIVER,
        DELIVER;
    }

    public static operatorStates state = operatorStates.IDLE;

    public enum pickupPositions{
        NONE,
        FLOOR,
        SHELF;
    }

    public static pickupPositions pickupPosition= pickupPositions.NONE;

    public static Boolean stored = false; //starting value

    public static double targetArmPosition = 0;


    public enum gamepieces {
        CONE,
        CUBE;
    }

    public static gamepieces gamepiece = gamepieces.CONE;

    static Boolean driverInput = false;
    public static void stateMachine(){
        System.out.println(state.toString());
        switch (state){
            case IDLE:


                //set all target positions to default (starting / default running position)
                targetArmPosition = 10;
                if(stored){
                    //wait for driver to select to progress to lining up and auto delivering
                    if(driverInput){
                        state = operatorStates.LINE_UP_APRILTAG;
                    }
                }
                else{
                    //wait for operator to have selected a viable pick up position
                    //handle operator inputs here, also need a confirmation button
                    //also need to handle operator selection of target object
                    if(pickupPosition != pickupPositions.NONE && driverInput){
                        state = operatorStates.MOVE_SUBSYSTEMS_PICKUP;
                    }
                }
                break;


            case LINE_UP:


                //
                break;


            case MOVE_SUBSYSTEMS_PICKUP:
                break;
            case INTAKE:
                break;
            case LINE_UP_APRILTAG:


                int pipeline;
                //enable automatic lining up using the swerve drive and the networktable values from the limelight subsystem
                //also select pipeline dependent on cone or cube
                if(gamepiece == gamepieces.CUBE){
                    pipeline = 1;
                }
                else{
                    pipeline = 2;
                }
                if(ComplexStateMachineDemo.demoSubsystemCondition){ //robot is within acceptable error margin of lined up
                    state = operatorStates.DETECT_DELIVERY;
                }
                break;


            case DETECT_DELIVERY:

                //we need to run the detection on the lined up apriltag, and scan from top to bottom to find the highest available slot
                //store a value: desired delivery height
                //waiting for the vision detection to be run
                //if it fails (no possible delivery zones then we go back to LINE_UP_APRILTAG, and taget another apriltag
                state = operatorStates.MOVE_SUBSYSTEMS_DELIVER;
                break;


            case MOVE_SUBSYSTEMS_DELIVER:

                //refer to which game piece is stored, refer to the target delivery position, determine which preset positions
                //output the target values to the subsystems
                //wait for the subsystems to be in the correct positions
                state = operatorStates.DELIVER;
                break;


            case DELIVER:

                //move outtake until no longer holding piece
                stored = false;
                state = operatorStates.IDLE;
                break;
        }
    }
}
