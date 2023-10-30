package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CyDogsSparky extends CyDogsChassis{

    private LinearOpMode myOpMode;
    public SpikeCam spikeCam;
    public Direction parkingSpot;
    private boolean isElbowOpen = false;

    public static final int ArmHomePosition = 5;
    public static final int ArmLow = 600;
    public static final int ArmMedium = 950;
    public static final int ArmHigh = 1300;
    public static final double ArmRaiseBeforeElbowMovement = 150;
    public static final double WristForIntake = 0.03;
    public static final double WristForDriving = 0.08;
    public static final double WristForScoring = 0.45;
    public static final double ElbowHomePosition = 0.49;
    public static final double ElbowScoringPosition = 0.23;
    public static final double FingerOpen = 0.8;
    public static final double FingerClosed = 0.5;
    public static final double DroneSecure = 0.53;
    public static final double DroneRelease = 0.3;
    public static final double CaptainReachHeight = 4000;
    public static final double CaptainPullHeight = 2000;

    public Servo Wrist;
    public DcMotor ArmLift;
    public Servo DroneReleaseServo;
    public Servo Elbow;
    public Servo Finger;
    public DcMotor Intake1;
    public DcMotor Intake2;
    public DcMotor TheCaptain;



    public CyDogsSparky(LinearOpMode currentOp) {
        super(currentOp);
        myOpMode = currentOp;
    }

    public void initializeSpikeCam(){
        spikeCam = new SpikeCam();
        spikeCam.initialize(myOpMode);
    }

    public void initializeDevices() {

        Wrist = myOpMode.hardwareMap.get(Servo.class, "Wrist");
        ArmLift = myOpMode.hardwareMap.get(DcMotor.class, "ArmLift");
        DroneReleaseServo = myOpMode.hardwareMap.get(Servo.class, "DroneRelease");
        Elbow = myOpMode.hardwareMap.get(Servo.class, "Elbow");
        Finger = myOpMode.hardwareMap.get(Servo.class, "Finger");
        Intake1 = myOpMode.hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = myOpMode.hardwareMap.get(DcMotor.class, "Intake2");
        TheCaptain = myOpMode.hardwareMap.get(DcMotor.class, "TheCaptain");


        // Initialize Intake
        Intake1.setDirection(DcMotor.Direction.REVERSE);
        Intake2.setDirection(DcMotor.Direction.FORWARD);
        Intake1.setPower(0);
        Intake2.setPower(0);
        Intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialize Drone Release and set position closed
        DroneReleaseServo.setDirection(Servo.Direction.FORWARD);
        // Initialize the Captain
        TheCaptain.setDirection(DcMotor.Direction.FORWARD);
        TheCaptain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TheCaptain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TheCaptain.setTargetPosition(0);
        TheCaptain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Initialize Arm Lift
        ArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setDirection(DcMotor.Direction.REVERSE);
        ArmLift.setPower(0.8);
        ArmLift.setTargetPosition(0);
        ArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Initialize Finger
        Finger.setDirection(Servo.Direction.FORWARD);
        // Initialize Wrist
        Wrist.setDirection(Servo.Direction.FORWARD);
        // Initialize Elbow
        Elbow.setDirection(Servo.Direction.REVERSE);
    }

    public void initializePositions() {
        DroneReleaseServo.setPosition(DroneSecure);
        Wrist.setPosition(WristForDriving);
        Elbow.setPosition(ElbowHomePosition);
        ArmLift.setTargetPosition(ArmHomePosition);
        Finger.setPosition(FingerClosed);
    }

    public void raiseArmToScore(int armHeight)
    {
        Finger.setPosition(FingerClosed);
        ArmLift.setPower(0.8);
        ArmLift.setTargetPosition(armHeight);
    }

    public void returnLiftForDriving()
    {
        ArmLift.setPower(0.6);
        ArmLift.setTargetPosition(ArmHomePosition);
    }

    public void openFinger()
    {
        Finger.setPosition(FingerOpen);
    }

    public void closeFinger()
    {
        Finger.setPosition(FingerClosed);
    }


    public void SwingElbow() {
        if (ArmLift.getCurrentPosition() > 590) {
            if (!isElbowOpen) {
                Elbow.setPosition(ElbowScoringPosition);
                Wrist.setPosition(WristForScoring);
                isElbowOpen = true;
            } else {
                Wrist.setPosition(WristForDriving);
                Elbow.setPosition(ElbowHomePosition);
                Finger.setPosition(FingerOpen);
                isElbowOpen = false;
            }
        }
    }

    public void scoreFromDrivingPositionAndReturn(int armHeight){
        raiseArmToScore(armHeight);
        myOpMode.sleep(1000);
        SwingElbow();
        myOpMode.sleep(2500);
        openFinger();
        myOpMode.sleep(500);
        SwingElbow();
        myOpMode.sleep(1500);
        returnLiftForDriving();
    }
    public void dropPurplePixel(){
        Intake1.setPower(-0.1);
        Intake2.setPower(-0.1);
        this.MoveStraight(20,0.5,500);
        Intake1.setPower(0);
        Intake2.setPower(0);
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
                    } else if (parkingSpot==Direction.RIGHT) {
                        myOpMode.telemetry.addLine("Parking RIGHT, Press Dpad Down to Confirm.");
                    } else {
                        myOpMode.telemetry.addLine("Nothing selected, press Right Bumper to restart selection.");
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

    public Direction askDrivePath(){
        Direction drivePath = null;

        if (myOpMode.opModeInInit()) {
            while (myOpMode.opModeInInit()) {
                // Put loop blocks here.
                while (drivePath==null) {
                    myOpMode.telemetry.addLine("Driver,");
                    myOpMode.telemetry.addLine("To drive through left, press DPAD LEFT");
                    myOpMode.telemetry.addLine("To drive through center, press DPAD UP");
                    myOpMode.telemetry.addLine("To drive through right, press DPAD RIGHT");
                    myOpMode.telemetry.update();
                    if (myOpMode.gamepad1.dpad_right) {
                        drivePath = Direction.RIGHT;
                        break;
                    }
                    if (myOpMode.gamepad1.dpad_left) {
                        drivePath = Direction.LEFT;
                        break;
                    }
                    if (myOpMode.gamepad1.dpad_up) {
                        drivePath = Direction.CENTER;
                        break;
                    }               }
                while (!myOpMode.gamepad1.dpad_down) {
                    if (drivePath==Direction.LEFT) {
                        myOpMode.telemetry.addLine("Driving under LEFT, Press Dpad Down to Confirm.");
                    } else if (drivePath==Direction.RIGHT) {
                        myOpMode.telemetry.addLine("Driving under RIGHT, Press Dpad Down to Confirm.");
                    } else if (drivePath==Direction.CENTER) {
                        myOpMode.telemetry.addLine("Driving under CENTER, Press Dpad Down to Confirm.");
                    } else {
                        myOpMode.telemetry.addLine("Nothing selected, press Right Bumper to restart selection.");
                    }
                    myOpMode.telemetry.update();
                    if (myOpMode.gamepad1.dpad_down) {
                        break;
                    }
                }
                if (drivePath==Direction.LEFT) {
                    myOpMode. telemetry.addLine("Driving under LEFT Confirmed.");
                    myOpMode.telemetry.update();
                    myOpMode.sleep(1000);
                } else if (drivePath==Direction.RIGHT) {
                    myOpMode. telemetry.addLine("Driving under RIGHT Confirmed.");
                    myOpMode.telemetry.update();
                    myOpMode.sleep(1000);
                } else if (drivePath==Direction.CENTER) {
                    myOpMode. telemetry.addLine("Driving under CENTER Confirmed.");
                    myOpMode.telemetry.update();
                    myOpMode.sleep(1000);
                } else {
                    myOpMode.telemetry.addLine("Driving under nopt selected.");
                    myOpMode.telemetry.update();
                }
                if (drivePath != null) {
                    break;
                }
                break;
            }

        }
        return drivePath;

    }
}