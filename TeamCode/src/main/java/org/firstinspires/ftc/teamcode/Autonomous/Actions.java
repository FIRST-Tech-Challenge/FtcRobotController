package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;

public class Actions {

 static Input input;
 static IMU imu;
 static ArmSynchronous a1;
 static HoldYawSync holdYaw;

 static boolean opModeisActive;

 public static void setupActions(Input localInput, ArmSynchronous actions1, IMU inertialMeasurementUnit, HoldYawSync holdYawSynchronized) {
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

   //holdYaw.setPos(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_YAW.getValue());

     while ((input.getTravelledDistance() < AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_DISTANCE.getValue()) && opModeisActive)  { //inches
        input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
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

     int pos =  AutoDistanceNumbers.HangSpecimen.ARM_HANG_POSITION.getValue();

     a1.setPos(pos);

     a1.extendArm(AutoDistanceNumbers.HangSpecimen.UPARM_HANG_POSITION.getValue(), 0);

     a1.setPos(AutoDistanceNumbers.HangSpecimen.ARM_SLIGHT_DOWN.getValue());

     a1.retractArm(AutoDistanceNumbers.HangSpecimen.UPARM_SLIGHT_BACK.getValue(), 0);


        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

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
                BotTelemetry.addData("location:", location);
                BotTelemetry.addData("samplea:", sampleX);
//                BotTelemetry.update();
                switch (sampleX) {
                    case 1:
                        BotTelemetry.addData("sampleb:", sampleX);
//                        BotTelemetry.update();
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_POS.getValue());
                        BotTelemetry.addData("distancea:", input.getTravelledDistance());
//                        BotTelemetry.update();
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromBar.DISTANCE_AWAY_BAR.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        BotTelemetry.addData("distanceb:", input.getTravelledDistance());
                        BotTelemetry.update();
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromBar.DISTANCE_TO_SAMPLE_1.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 2:
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_POS.getValue());
                        while (input.getTravelledDistance() < 20) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromBar.DISTANCE_TO_SAMPLE_2.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_POS.getValue());
                        while (input.getTravelledDistance() < 20) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < 25) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 3:
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_POS.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromBar.DISTANCE_AWAY_BAR.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromBar.DISTANCE_TO_SAMPLE_3.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromBar.IMU_TURN_POS.getValue());
                        input.resetDistance();
                        break;
                    default:
                        break;
                }
                BotTelemetry.addData("before Break:", 0);
                BotTelemetry.update();
            break;

            case "observation":
                switch (sampleX) {
                    case 1:
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromObserve.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromObserve.DISTANCE_TO_SAMPLE_1.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 2:
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromObserve.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromObserve.DISTANCE_TO_SAMPLE_2.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                        }
                        input.move(0);
                        input.resetDistance();
                        break;
                    case 3:
                        holdYaw.setPos(AutoDistanceNumbers.DriveBehindSampleFromObserve.IMU_TURN_TO_SAMPLE.getValue());
                        while (input.getTravelledDistance() < AutoDistanceNumbers.DriveBehindSampleFromObserve.DISTANCE_TO_SAMPLE_3.getValue()) {
                            input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
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
                holdYaw.setPos(-90);

                while ((input.getTravelledDistance() < 10) && opModeisActive)  { //inches
                    input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                }
                input.move(0);
                input.resetDistance();
                holdYaw.setPos(AutoDistanceNumbers.PushAndReverse.IMU_TURN_TO_SAMPLE.getValue());
                //Move backwards aka behind samples
                while ((input.getTravelledDistance() < AutoDistanceNumbers.PushAndReverse.DISTANCE_TO_OBSERVATION.getValue()) && opModeisActive)  { //inches
                    input.move(AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                }
                input.move(0);
                input.resetDistance();
                break;
            case "reverse":
                holdYaw.setPos(AutoDistanceNumbers.PushAndReverse.IMU_FACE_BACK.getValue());
                //Move backwards aka behind samples
                while ((input.getTravelledDistance() > -AutoDistanceNumbers.PushAndReverse.DISTANCE_TO_OBSERVATION.getValue()) && opModeisActive)  { //inches
                    input.move(-AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
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
        a1.setPos(AutoDistanceNumbers.GrabSpecimen.ARM_START_POS.getValue());
        input.claw(true,false);
        input.automaticallyMoveWrist(false);

        try {
            Thread.sleep(AutoDistanceNumbers.GrabSpecimen.WAIT_TIME_MILLISECOND.getValue());
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        input.automaticallyMoveWrist(false);
        input.claw(false,true);
        input.automaticallyMoveWrist(false);
        a1.setPos(0);
    }

    public static void driveToInitialPositionFromHumanPlayerPitStopX (int pitstop) {
        switch (pitstop) {
            case 1:
                //drive to initial pos
                holdYaw.setPos(AutoDistanceNumbers.DriveToInitialPos.IMU_FACE_INIT_POS.getValue());
                while ((input.getTravelledDistance() < AutoDistanceNumbers.DriveToInitialPos.DISTANCE_FROM_PITSTOP_1.getValue()) && opModeisActive) { //inches
                    input.move(-AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                }
                input.move(0);
                input.resetDistance();
                holdYaw.setPos(0);
                break;
            case 2:
                //drive to initial pos
                holdYaw.setPos(AutoDistanceNumbers.DriveToInitialPos.IMU_FACE_INIT_POS.getValue());
                while ((input.getTravelledDistance() < AutoDistanceNumbers.DriveToInitialPos.DISTANCE_FROM_PITSTOP_2.getValue()) && opModeisActive) { //inches
                    input.move(-AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
                }
                input.move(0);
                input.resetDistance();
                holdYaw.setPos(0);
                break;
            case 3:
                //drive to initial pos
                holdYaw.setPos(AutoDistanceNumbers.DriveToInitialPos.IMU_FACE_INIT_POS.getValue());
                while ((input.getTravelledDistance() < AutoDistanceNumbers.DriveToInitialPos.DISTANCE_FROM_PITSTOP_3.getValue()) && opModeisActive) { //inches
                    input.move(-AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
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
        holdYaw.setPos(AutoDistanceNumbers.DriveToObserve.IMU_FACE_INIT_POS.getValue());
        while ((input.getTravelledDistance() < AutoDistanceNumbers.DriveToObserve.DISTANCE_TO_OBSERVE.getValue()) && opModeisActive)  { //inches
            input.move(-AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
        }
        input.move(0);
        input.resetDistance();
    }

    public static void park () {
        //drive to observation zone from the bars
        holdYaw.setPos(AutoDistanceNumbers.Park.IMU_FACE_PARK.getValue());
        while ((input.getTravelledDistance() < AutoDistanceNumbers.Park.DISTANCE_TO_OBSERVE.getValue()) && opModeisActive)  { //inches
            input.move(-AutoDistanceNumbers.DriveForward.DRIVE_FORWARD_POWER.getValue());
        }
    }


}
