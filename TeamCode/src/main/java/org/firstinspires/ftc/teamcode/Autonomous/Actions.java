package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

public class Actions {

 static Input input;

 static boolean opModeisActive;

 public static void setupActions(Input localInput) {
   input = localInput;
 }

 public static void opModeIsActive (boolean localOpModeisActive){
   opModeisActive = localOpModeisActive;
 }

   int SPECIMEN_OFFSET = 20;


    public static void driveToBarFromInitialPositionForSpecimen(int specimen) {


     while ((input.getTravelledDistance() < 17) && opModeisActive)  { //inches
       input.move(-20);
     }
     input.move(0);
        // 1. define and hardcode initial position (eg A1)
        // 2. define and hardcode bar location (eg A5)
        // 3. measure distance between initial position and bar location
        // 4. measure and hardcode offset for each specimen
        // 5. Robot should stop there when the arm position is exactly ready to hang
        // 6. for each specimen location := specimenX * offset
        // 7. always save robot position
    }

    public static void hangAndReleaseSpecimen() {

     //while (input.getArmPos() < 1065 && opModeisActive) {
      input.arm(1465);
     //}
//
//     //uparm until -2750
     while (input.getArmPos() >= 1465) {
      while (input.getUpArmPos() > -2550 && opModeisActive) {
       input.upArm(-50);
      }

      input.upArm(0);
     }

//     while (input.getUpArmPos() <= -2550 && input.getArmPos() < 1439 && opModeisActive) {
 //     input.arm(1439);
//     }
//
//     while (input.getUpArmPos() < -2070 && opModeisActive) {
//      input.upArm(50);
//     }
//     input.upArm(0);


     //wait a bit
     //then arm 1548
     //wait a bit
     //up arm -1675
     //wait a bit
//     //let go
//     //reverse everything
//     input.claw(true,false);
//
//
//     BotTelemetry.addData("armPosition", input.getArmPos());
   BotTelemetry.update();

        // 1. robot.lower.arm and press down to hang
        // 2. robot.release claw
        // 3. robot.retract.arm
    }

    public static void driveBehindSampleFromLocation(int sampleX, String location) {
        // String location can either bar or human player
        // 1. measure distance from bar to behind sampleX and hardcode the value
        // ..
        // ..
        // if (location == human player) {
        //   reverse pushSampleToHumanPlayer(int sampleX)
        // }
    }

    public static void pushOrReverseSampleToHumanPlayer(int sampleX, String pushOrReverse) {
        // 1. measure distance from behind sample to human player
        // 2. stop there where arm is exactly ready to grab the specimen from the wall
        // 3. If push, robot.forward (wheel power +)
        // 4. if reverse, robot.reverse (wheel power -)
    }

    public static void driveRightUntilBehindSample(int sampleX) {
        // distance between samples is 10inch
        // measure how many wheel strafes needed for 10 inch
    }


    public static void grabSpecimen(int specimen) {
        // 1 extend arm
        // 2 grab with claw
    }

    public static void driveToInitialPositionFromHumanPlayerPitStopX (int pitstop) {
        //drive to initial pos
    }

    public static void driveToHumanPlayerFromBarForFinalSpecimen() {
        //drive back
    }

    public static void levelOneAsension () {
        //arm touches bar
    }


}
