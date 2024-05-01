package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

// CyDogsSparky is the class that represents the Sparky robot for the 2023 robotics year.
//   it contains all the functionality needed to do the basic activities on the robot.
//   It is designed to be instantiated in the op mode programs we write.  It inherits from
//   CyDogsChassis which is our basic chassis code.
public class CyDogsSparky extends CyDogsChassis{

    private LinearOpMode myOpMode;
    public SpikeCam spikeCam;
    public Direction parkingSpot;

    // this lets us swing the elbow back and forth by keeping track if it is open or closed
    private boolean isElbowOpen = false;

    public Alliance myAlliance;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    // This section is all of our settings for our servos.  We code them here once and use the
    //   variable names so that we only have them stored in one place in case we need to change them.
    public static final int ArmHomePosition = 0;
    public static final int ArmLow = 1800;
    public static final int ArmMedium = 3900;
    public static final int ArmHigh = 6300;
    public static final int ArmRaiseBeforeElbowMovement = 3400;
    public static final double WristForDriving = 0.46;
    public static final double WristForScoring = 0.76;
    public static final double ElbowHomePosition = 0.51;
    public static final double ElbowScoringPosition = 0.76;
    public static final double FingerLeftOpen = 0.4;
    public static final double FingerLeftClosed = 0.5;
    public static final double FingerRightOpen = 0.4;
    public static final double FingerRightClosed = 0.53;


    public static final int BackUpDistanceFromSpike = 30;
    public static final int DistanceBetweenScoreBoardAprilTags = 150;

    // We have a common standard auton wait time built in so that we can tweak the overall speed
    //  of the auton when needed.  A parameter is in the constructor call to set this.
    public int StandardAutonWaitTime = 500;


    public Servo Wrist;
    public DcMotor ArmLift;
    public Servo DroneReleaseServo;
    public Servo Elbow;
    public Servo FingerLeft;
    public Servo FingerRight;


    // This is the constructor, passing in the current op mode, what alliance we are, and what
    //   standard wait time we want.
    public CyDogsSparky(LinearOpMode currentOp, Alliance currentAlliance, int standardWaitTime) {
        // this line also ensures the constructor for CyDogsChassis also gets called
        super(currentOp);

        myAlliance = currentAlliance;
        myOpMode = currentOp;
        StandardAutonWaitTime = standardWaitTime;
    }

    public void initializeSpikeCam(){
        spikeCam = new SpikeCam();
        WebcamName webcam1 = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera spikeCamera = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
        spikeCam.initialize(myOpMode, myAlliance, spikeCamera);
    }

    public void initializeDevices() {

        Wrist = myOpMode.hardwareMap.get(Servo.class, "Wrist");
        ArmLift = myOpMode.hardwareMap.get(DcMotor.class, "ArmLift");
        //  Note this was added in prep for moving Teleop to Java
        DroneReleaseServo = myOpMode.hardwareMap.get(Servo.class, "DroneRelease");
        Elbow = myOpMode.hardwareMap.get(Servo.class, "Elbow");
        FingerLeft = myOpMode.hardwareMap.get(Servo.class, "FingerLeft");
        FingerRight = myOpMode.hardwareMap.get(Servo.class, "FingerRight");


        // Initialize Drone Release and set position closed
        DroneReleaseServo.setDirection(Servo.Direction.FORWARD);

        // Initialize Arm Lift
        ArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setDirection(DcMotor.Direction.FORWARD);
        ArmLift.setPower(0.9);
        ArmLift.setTargetPosition(0);
        ArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Initialize Finger
        FingerLeft.setDirection(Servo.Direction.FORWARD);
        FingerRight.setDirection(Servo.Direction.FORWARD);
        // Initialize Wrist
        Wrist.setDirection(Servo.Direction.FORWARD);
        // Initialize Elbow
        Elbow.setDirection(Servo.Direction.FORWARD);
    }

    public void initializePositions() {
        //DroneReleaseServo.setPosition(DroneSecure);
        Wrist.setPosition(WristForDriving);
        Elbow.setPosition(ElbowHomePosition);
        ArmLift.setTargetPosition(ArmHomePosition);
        FingerLeft.setPosition(FingerLeftClosed);
        FingerRight.setPosition(FingerRightClosed);
    }

    public void initializeAprilTags()
    {
        WebcamName webcam2 = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam2)
                .addProcessor(aprilTag)
                .build();
    }

    public void raiseArmToScore(int armHeight)
    {
    //    Finger.setPosition(FingerClosed);
        ArmLift.setPower(0.8);
        ArmLift.setTargetPosition(armHeight);
    }

    public void returnLiftForDriving()
    {
        ArmLift.setPower(0.8);
        ArmLift.setTargetPosition(ArmHomePosition);
    }

    // this is designed to force the arm down at the end of autonomous in an attempt to ensure it's
    //   all the way down
    public void LowerArmAtAutonEnd()
    {
        ArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmLift.setPower(-0.4);
        myOpMode.sleep(1000);
        ArmLift.setPower(0);

    }

    public void openFingers()
    {
        FingerLeft.setPosition(FingerLeftOpen);
        FingerRight.setPosition(FingerRightOpen);
    }



    public void SwingElbow() {
        // check to see if the arm is high enough before moving the elbow.  Do nothing if it is not.
        //   ROBOT MAY BREAK if elbow moves when arm is too low.  The -20 is a small amount that
        //   compensates for if the motor didn't get exactly to the right spot.
        if (ArmLift.getCurrentPosition() > ArmRaiseBeforeElbowMovement-20) {
            if (!isElbowOpen) {
             //   myOpMode.telemetry.addData("Elbow is not open, trying to open",ArmLift.getCurrentPosition());
             //   myOpMode.telemetry.update();
             //   myOpMode.sleep(3000);

                Elbow.setPosition(ElbowScoringPosition);
                Wrist.setPosition(WristForScoring);
                isElbowOpen = true;
            } else {
             //   myOpMode.telemetry.addData("Elbow is open, trying to close",ArmLift.getCurrentPosition());
              //  myOpMode.telemetry.update();
              //  myOpMode.sleep(3000);

                Wrist.setPosition(WristForDriving);
                Elbow.setPosition(ElbowHomePosition);
                FingerLeft.setPosition(FingerLeftOpen);
                FingerRight.setPosition(FingerRightOpen);
                isElbowOpen = false;
            }
        }
        else {
            myOpMode.telemetry.addData("Arm not high enough, arm at:",ArmLift.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.sleep(3000);
        }
    }

    public void scoreFromDrivingPositionAndReturn(){

        SwingElbow();
        myOpMode.sleep(800);
        raiseArmToScore(1650);
        myOpMode.sleep(2000);
        openFingers();
        myOpMode.sleep(400);
    }


    public void returnArmFromScoring(){
        raiseArmToScore(ArmMedium);
        myOpMode.sleep(1700);
        SwingElbow();
        myOpMode.sleep(1500);
        returnLiftForDriving();
        myOpMode.sleep(2000);
    }

    public void AdjustToAprilTag(SpikeCam.location mySpike, String corner)
    {

        if(mySpike==SpikeCam.location.LEFT) {
            StrafeLeft(DistanceBetweenScoreBoardAprilTags, .5, 300);
        } else if (mySpike==SpikeCam.location.RIGHT) {
            StrafeRight(DistanceBetweenScoreBoardAprilTags+130,.5,300);
        }
        int targetTag = getAprilTagTarget(mySpike, myAlliance);
       // myOpMode.sleep(400);

        double degreesBearing;
        double inchesXMovement;
        double inchesYMovement;
        // Adjust once
        AprilTagDetection foundTag = GetAprilTag(targetTag);
        // X
     //   if(foundTag != null) {
    //        inchesXMovement = foundTag.ftcPose.x;
     //       StrafeRight((int) (-(inchesXMovement)* 25.4), .5, 400);
     //   }
        // Bearing
     //   if(foundTag != null) {
      //      degreesBearing = foundTag.ftcPose.bearing;
      //      RotateRight((int)-degreesBearing,.5,400);
     //   }


        // Adjust a 2nd Time
      // foundTag = GetAprilTag(targetTag);
        // Bearing

     //   myOpMode.sleep(400);
     //   foundTag = GetAprilTag(targetTag);
        // X
        double extraInches =0;
 //       if(targetTag==6)
 //       {
 //           extraInches += 1.5;
 //       }
 //       if(targetTag==3){
 //           extraInches -= 0.0;
 //       }
        if(foundTag != null) {
            inchesXMovement = foundTag.ftcPose.x;

            StrafeRight((int) (-(inchesXMovement +extraInches)* 25.4), .5, 400);
        }

    //    foundTag = GetAprilTag(targetTag);
      //  if(foundTag != null) {
      //      degreesBearing = foundTag.ftcPose.bearing;
            //       if(targetTag==3)
            //     {
            //       degreesBearing = degreesBearing*.9;
            // }


      //      RotateRight((int)(-degreesBearing*.8),.5,300);
     //   }

        // Y
        double adjustY = 6.5;
    //    if(targetTag==2)
      //  {
        //    adjustY = 6.5;

        //}
   //     myOpMode.sleep(400);
   //     foundTag = GetAprilTag(targetTag);
        if(foundTag != null) {
            inchesYMovement = foundTag.ftcPose.y;
            MoveStraight((int) (inchesYMovement * 25.4-adjustY*25.4), .5, 400);
            myOpMode.telemetry.addData("Target: ", getAprilTagTarget(mySpike, myAlliance));
            myOpMode.telemetry.addData("Bearing: ", foundTag.ftcPose.bearing);
            myOpMode.telemetry.addData("X adjustment: ", foundTag.ftcPose.x);
            myOpMode.telemetry.addData("Y adjustment: ", foundTag.ftcPose.y);
            myOpMode.telemetry.update();
        //    myOpMode.sleep(7000);
        }

        if(foundTag ==null) {
                MoveStraight(315,.5,450);
        } else {
  /*          if (targetTag == 1 && corner == "BlueRight") {
                MoveStraight(7, .5, 400);
            }
            if (targetTag == 3 && corner == "BlueLeft") {
                MoveStraight(7, .5, 400);
            }
            if (targetTag == 3 && corner == "BlueRight") {
                StrafeRight(40, .5, 400);
            }
            if (targetTag == 4 && corner == "RedLeft") {
                StrafeLeft(30, .5, 400);
            }
            if (targetTag == 5 && corner == "RedLeft") {
                StrafeRight(20, .5, 400);
            }
            if (targetTag == 6 && corner == "RedRight") {
                StrafeRight(25, .5, 400);
            }
   */     }

     //     myOpMode.sleep(8000);
    }

    public int getAprilTagTarget(SpikeCam.location mySpike, Alliance myAlliance)
    {
        int targetTag;
        if(myAlliance==Alliance.RED)
        {
            if(mySpike==SpikeCam.location.LEFT)
            {
                targetTag=4;
            } else if (mySpike== SpikeCam.location.MIDDLE) {
                targetTag=5;
            }
            else {   // RIGHT
                targetTag=6;
            }
        }
        else
        {
            if(mySpike==SpikeCam.location.LEFT)
            {
                targetTag=1;
            } else if (mySpike== SpikeCam.location.MIDDLE) {
                targetTag=2;
            }
            else {   // RIGHT
                targetTag=3;
            }
        }
        return targetTag;
    }

    public void dropPurplePixel(){
        FingerLeft.setPosition(FingerLeftOpen);
     //   FingerRight.setPosition(FingerRightOpen);
     //   this.MoveStraight(BackUpDistanceFromSpike,0.5,500);
    }

    public Direction askParkingSpot(){
        Direction parkingSpot = null;

        if (myOpMode.opModeInInit()) {
            while (myOpMode.opModeInInit()) {
                // Put loop blocks here.
                while (parkingSpot==null) {
                    myOpMode.telemetry.addLine("Driver,");
                    myOpMode.telemetry.addLine("To park LEFT of the backboard, press DPAD LEFT");
                    myOpMode.telemetry.addLine("To park RIGHT of the backboard, press DPAD RIGHT");
                    myOpMode.telemetry.update();
                    if (myOpMode.gamepad1.dpad_right) {
                       parkingSpot = Direction.RIGHT;
                        break;
                    }
                    if (myOpMode.gamepad1.dpad_left) {
                        parkingSpot = Direction.LEFT;
                        break;
                    }
                }
                while (!myOpMode.gamepad1.dpad_down) {
                    if (parkingSpot==Direction.LEFT) {
                        myOpMode.telemetry.addLine("Parking LEFT, Press Dpad Down to Confirm.");
                        myOpMode.telemetry.addLine("Press Dpad Up to restart selection.");
                    } else if (parkingSpot==Direction.RIGHT) {
                        myOpMode.telemetry.addLine("Parking RIGHT, Press Dpad Down to Confirm.");
                        myOpMode.telemetry.addLine("Press Dpad Up to restart selection.");
                   // } else {
                        //myOpMode.telemetry.addLine("Nothing selected, press Right Bumper to restart selection.");
                    }
                    myOpMode.telemetry.update();
                    if (myOpMode.gamepad1.dpad_down) {
                        break;
                    }
                }
                if (parkingSpot==Direction.LEFT) {
                    myOpMode. telemetry.addLine("Parking LEFT Confirmed.");
                    myOpMode.telemetry.update();
                    myOpMode.sleep(1000);
                } else if (parkingSpot==Direction.RIGHT) {
                    myOpMode. telemetry.addLine("Parking RIGHT Confirmed.");
                    myOpMode.telemetry.update();
                    myOpMode.sleep(1000);
                } else {
                    myOpMode.telemetry.addLine("Nothing selected.");
                    myOpMode.telemetry.update();
                }
                if (parkingSpot != null) {
                    break;
                }
                break;
            }

        }
        return parkingSpot;

    }

   // public Direction askDrivePath(){
      //  Direction drivePath = null;

      //  if (myOpMode.opModeInInit()) {
         //   while (myOpMode.opModeInInit()) {
                // Put loop blocks here.
             //   while (drivePath==null) {
                 //   myOpMode.telemetry.addLine("Driver,");
                 //   myOpMode.telemetry.addLine("To drive through left, press DPAD LEFT");
             //       myOpMode.telemetry.addLine("To drive through center, press DPAD UP");
                 //  myOpMode.telemetry.addLine("To drive through right, press DPAD RIGHT");
                 //   myOpMode.telemetry.update();
                 //   if (myOpMode.gamepad1.dpad_right) {
                  //      drivePath = Direction.RIGHT;
                    //    break;
                  //  }
                  //  if (myOpMode.gamepad1.dpad_left) {
                    //   drivePath = Direction.LEFT;
                     //   break;
                  //  }
                //    if (myOpMode.gamepad1.dpad_up) {
                //        drivePath = Direction.CENTER;
                //        break;
                 //   }
              //  }
              //  while (!myOpMode.gamepad1.dpad_down) {
                  //  if (drivePath==Direction.LEFT) {
                   //     myOpMode.telemetry.addLine("Driving under LEFT, Press Dpad Down to Confirm.");
                  //  } else if (drivePath==Direction.RIGHT) {
                    //   myOpMode.telemetry.addLine("Driving under RIGHT, Press Dpad Down to Confirm.");
                  //  } else if (drivePath==Direction.CENTER) {
                     //   myOpMode.telemetry.addLine("Driving under CENTER, Press Dpad Down to Confirm.");
                  //  } else {
                      //  myOpMode.telemetry.addLine("Nothing selected, press Right Bumper to restart selection.");
                 //  }
                  //  myOpMode.telemetry.update();
                   // if (myOpMode.gamepad1.dpad_down) {
                    //    break;
                 //  }
              //  }
              //  if (drivePath==Direction.LEFT) {
                 //   myOpMode. telemetry.addLine("Driving under LEFT Confirmed.");
                 //   myOpMode.telemetry.update();
                //    myOpMode.sleep(1000);
             //  } else if (drivePath==Direction.RIGHT) {
               //     myOpMode. telemetry.addLine("Driving under RIGHT Confirmed.");
                //   myOpMode.telemetry.update();
                 //   myOpMode.sleep(1000);
             //   } else if (drivePath==Direction.CENTER) {
                //   myOpMode. telemetry.addLine("Driving under CENTER Confirmed.");
                 //  myOpMode.telemetry.update();
                 //   myOpMode.sleep(1000);
              //  } else {
                  //  myOpMode.telemetry.addLine("Driving under not selected.");
                  //  myOpMode.telemetry.update();
               // }
              //  if (drivePath != null) {
               //     break;
              //  }
              //  break;
           // }

      //  }
      //  return drivePath;

   //}

    public void AutonPlacePurplePixel(SpikeCam.location mySpike){
        if(mySpike==SpikeCam.location.LEFT){
            RotateLeft(94,.5,StandardAutonWaitTime);
            MoveStraight(-15,.5,200);
           // MoveStraight(-20,.5,200);
            dropPurplePixel();
        } else if (mySpike==SpikeCam.location.MIDDLE) {
            MoveStraight(70,.5,StandardAutonWaitTime);
            dropPurplePixel();
        } else { //Right
            RotateLeft(-90,.5,StandardAutonWaitTime);
         //   MoveStraight(-10,.5,200);
            dropPurplePixel();
        }
    }



    public void AutonCenterOnScoreboardBasedOnPath(Direction myPath) {
        if(myPath==Direction.LEFT) {
            StrafeRight(OneTileMM,.5,StandardAutonWaitTime);
        } else if (myPath==Direction.CENTER) {
            // Should be aligned in the center already
        } else {
            StrafeLeft(OneTileMM-90,.5,StandardAutonWaitTime);
        }
    }

    public void AutonParkInCorrectSpot(SpikeCam.location mySpike, Direction myParkingSpot){
        int leftAdjustment = 0;
        int rightAdjustment = 0;

        // need to adjust distance based on which april tag we're in front of
        if(mySpike==SpikeCam.location.LEFT) {
            leftAdjustment = -DistanceBetweenScoreBoardAprilTags;
            rightAdjustment = DistanceBetweenScoreBoardAprilTags;
        } else if (mySpike==SpikeCam.location.MIDDLE) {
            // no adjustments needed
        } else {
            leftAdjustment = DistanceBetweenScoreBoardAprilTags;
            rightAdjustment = -DistanceBetweenScoreBoardAprilTags;
        }

        if(myParkingSpot==Direction.LEFT){
            StrafeLeft(OneTileMM+leftAdjustment+120,.7,StandardAutonWaitTime);
        } else if (myParkingSpot==Direction.CENTER) {
            // really shouldn't park in center, but if so, I guess we're here
        } else {
            StrafeRight(OneTileMM+rightAdjustment+120,.7,StandardAutonWaitTime);
        }
    }

    public AprilTagDetection GetAprilTag(int tagID)
    {
        telemetryAprilTag();
        // Get the latest AprilTag detections from the pipeline.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Iterate over the detections and return the one that matches the specified ID.
        for (AprilTagDetection aprilTagDetection : currentDetections) {
            if (aprilTagDetection.id == tagID) {
                return aprilTagDetection;
            }
        }

        // If no matching detection is found, return null.
        return null;
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        myOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }


        }   // end for() loop

        // Add "key" information to telemetry
        //     myOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //    myOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //     myOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
        //    myOpMode.telemetry.update();

    }   // end method telemetryAprilTag()
}