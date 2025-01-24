package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Actions {

 static Input input;
 static IMU imu;
 static Action1 a1;

 static boolean opModeisActive;

 public static void setupActions(Input localInput, Action1 actions1, IMU inertialMeasurementUnit) {
   input = localInput;
   a1 = actions1;
   imu = inertialMeasurementUnit;
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

     int pos =  1265;

     a1.setPos(pos);

     a1.extendArm(-2550, 0);

     pos = 1639;
     a1.setPos(pos);

     a1.retractArm(-1500, 1639);

     input.claw(true,false);

     pos = 0;
     a1.setPos(pos);

     a1.retractArm(-120, 0);


//     while (input.getArmPos() < 1465) {
//      input.arm(1465);
//     }

//     ExecutorService executor = Executors.newSingleThreadExecutor();
    // ExecutorService executor1 = Executors.newSingleThreadExecutor();
//     boolean step1Done = false;

     //while (opModeisActive && !step1Done) {
      //input.arm(1265);

//      while (input.getUpArmPos() > -2550 && opModeisActive) {
//       input.upArm(-50);
//      }
//       input.upArm(0);
      //}
    // }
//
//     CompletableFuture<String> future = CompletableFuture.supplyAsync(() -> {
//          while (opModeisActive) {
//           input.arm(1265);
//          }
//
//          return "Hello from the future!";
//     }, executor);
//
//        //uparm until -2750
//
//      while (input.getUpArmPos() < -1550 && opModeisActive) {
//       input.upArm(50);
//      }
//
//      input.upArm(0);
//
//
//     //future.thenAccept(result -> System.out.println(result));
//     executor.shutdownNow();
//        try {
//            future.get();
//        } catch (ExecutionException e) {
//            throw new RuntimeException(e);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        executor.shutdown();


//     CompletableFuture<String> future1 = CompletableFuture.supplyAsync(() -> {
//         while (opModeisActive) {
//          input.arm(1439);
//         }
//
//      return "Hello from the future!";
//     }, executor1);
//
//     future1.thenAccept(result -> System.out.println(result));
//     executor1.shutdown();

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
  // BotTelemetry.update();

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

    public static void park () {
        //drive to observation zone from the bars

     //this way (turn right, move, turn right) or turn like 45 or 60 degrees then just move forward will reach same destination
     while (imu.getAngle() > 88 && imu.getAngle() < 92) { //turn 90 degrees (right) without a pid loop which would be more precise so just turn until in a range
       input.spin(20);
     }

     while ((input.getTravelledDistance() < 25) && opModeisActive)  { //inches
      input.move(20);
     }

     while (imu.getAngle() > 88 && imu.getAngle() < 92) { //turn 90 degrees (right)
      input.spin(20);
     }

     while ((input.getTravelledDistance() < 25) && opModeisActive)  { //inches
      input.move(20);
     }

     // done!

    }


}
