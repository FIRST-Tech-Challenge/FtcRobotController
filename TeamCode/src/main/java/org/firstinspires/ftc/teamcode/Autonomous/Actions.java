package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;

public class Actions {

 static Input input;
 static IMU imu;
 static armSynchronous a1;
 static holdYawSync holdYaw;

 static boolean opModeisActive;

 public static void setupActions(Input localInput, armSynchronous actions1, IMU inertialMeasurementUnit, holdYawSync holdYawSynchronized) {
   input = localInput;
   a1 = actions1;
   imu = inertialMeasurementUnit;
   holdYaw = holdYawSynchronized;
 }

 public static void opModeIsActive (boolean localOpModeisActive){
   opModeisActive = localOpModeisActive;
 }


   int SPECIMEN_OFFSET = 20;


    public static void driveToBarFromInitialPositionForSpecimen() {

    holdYaw.setPos(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_YAW.getValue());

     while ((input.getTravelledDistance() < AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_DISTANCE.getValue()) && opModeisActive)  { //inches
        input.move(-20);
     }
     input.move(0);
     input.resetDistance();
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

     input.automaticallyMoveWrist(true);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    input.automaticallyMoveWrist(false);

    input.claw(true,false);

     a1.retractArm(-120, 0);

     pos = 0;
     a1.setPos(pos);

    }

    public static void driveBehindSampleFromLocations(int sampleX, String location) {
        /** String location can either bar or human player
        // 1. measure distance from bar to behind sampleX and hardcode the value
        // ..
        // ..
        // if (location == human player) {
        //   reverse pushSampleToHumanPlayer(int sampleX)
        // }
       // reverse to this pos
         */
        switch (location) {
            case "bar":
                switch (sampleX) {
                    case 1:
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 20) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(0);
                        while (input.getTravelledDistance() < 20) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 2:
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 20) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(0);
                        while (input.getTravelledDistance() < 25) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 20) {
                            input.move(-20);
                        }
                        input.move(180);
                        input.resetDistance();
                        holdYaw.setPos(0);
                        while (input.getTravelledDistance() < 25) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 3:
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 20) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(0);
                        while (input.getTravelledDistance() < 30) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 25) {
                            input.move(-20);
                        }
                        input.move(180);
                        input.resetDistance();
                        holdYaw.setPos(0);
                        while (input.getTravelledDistance() < 30) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    default:
                        break;
                }
            break;

            case "observation":
                switch (sampleX) {
                    case 1:
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 20) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 2:
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 25) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 3:
                        holdYaw.setPos(90);
                        while (input.getTravelledDistance() < 30) {
                            input.move(-20);
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }


//        switch (location) {
//         case "bar":
//
//            ElapsedTime runtime = new ElapsedTime();
//            runtime.reset();
//
//            //Move to the right
//            double holdTime;
//            while (opModeisActive && (runtime.milliseconds() < 1000)) {
//                input.strafe(50);
//            }
//            input.strafe(0);
//
//            //Move forward
//            while ((input.getTravelledDistance() < 30) && opModeisActive)  { //inches
//             input.move(-20);
//            }
//            input.move(0);
//            input.resetDistance();
//
//           //Move to the right
//           runtime.reset();
//           while (opModeisActive && (runtime.milliseconds() < 300.0)) {
//              input.strafe(50);
//           }
//           input.strafe(0);
//           break;
//         case "x":
//          break;
//         default:
//          break;
//      }






    }

    public static void pushOrReverseSampleToHumanPlayer(String pushOrReverse) {
        /**
        // 1. measure distance from behind sample to human player
        // 2. stop there where arm is exactly ready to grab the specimen from the wall
        // 3. If push, robot.forward (wheel power +)
        // 4. if reverse, robot.reverse (wheel power -)
         */
        switch (pushOrReverse) {
            case "push":
                holdYaw.setPos(180);
                //Move backwards aka behind samples
                while ((input.getTravelledDistance() < 84) && opModeisActive)  { //inches
                    input.move(-20);
                }
                input.move(0);
                input.resetDistance();
                break;
            case "reverse":
                holdYaw.setPos(180);
                //Move backwards aka behind samples
                while ((input.getTravelledDistance() > -84) && opModeisActive)  { //inches
                    input.move(20);
                }
                input.move(0);
                input.resetDistance();
                break;
            default:
                break;
        }


    }

    public static void driveRightUntilBehindSample() {
        holdYaw.setPos(90);
        //Move backwards aka behind samples
        while ((input.getTravelledDistance() < 10) && opModeisActive)  { //inches
            input.move(-20);
        }
        input.move(0);
        input.resetDistance();
        holdYaw.setPos(0);
    }


    public static void grabSpecimen(int specimen){
        // 1 extend arm
        // 2 grab with claw
        a1.setPos(2000);
        input.claw(true,false);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        input.claw(false,true);
        a1.setPos(0);
    }

    public static void driveToInitialPositionFromHumanPlayerPitStopX (int pitstop) {
        switch (pitstop) {
            case 1:
                //drive to initial pos
                holdYaw.setPos(270);
                while ((input.getTravelledDistance() < 30) && opModeisActive) { //inches
                    input.move(-20);
                }
                input.move(0);
                input.resetDistance();
                holdYaw.setPos(0);
                break;
            case 2:
                //drive to initial pos
                holdYaw.setPos(270);
                while ((input.getTravelledDistance() < 40) && opModeisActive) { //inches
                    input.move(-20);
                }
                input.move(0);
                input.resetDistance();
                holdYaw.setPos(0);
                break;
            case 3:
                //drive to initial pos
                holdYaw.setPos(270);
                while ((input.getTravelledDistance() < 50) && opModeisActive) { //inches
                    input.move(-20);
                }
                input.move(0);
                input.resetDistance();
                holdYaw.setPos(0);
                break;
            default:
                break;
        }
    }

    public static void driveToHumanPlayerFromBarForFinalSpecimen() {
        //drive back
        holdYaw.setPos(45);
        while ((input.getTravelledDistance() < 30) && opModeisActive)  { //inches
            input.move(-20);
        }
        input.move(0);
        input.resetDistance();
    }

    public static void park () {
        //drive to observation zone from the bars
        holdYaw.setPos(225);
        while ((input.getTravelledDistance() < 30) && opModeisActive)  { //inches
            input.move(-20);
        }
    }


}
