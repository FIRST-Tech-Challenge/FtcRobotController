package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.auto.CSBaseLIO;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.OuttakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DroneStateMachine;
import org.firstinspires.ftc.teamcode.team.states.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.HangStateMachine;


/*
 * This {@code class} acts as the driver-controlled program for FTC team 16598 for the CenterStage
 * challenge. By extending {@code CSRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *
 *  User 1: (Ronnie)
 *      Drive:
 *          Left & Right joysticks     -> Mecanum drive
 *          Left-Bumper                -> Decrease robot speed .7x
 *          Right-Bumper               -> Normal robot speed 1x
 *
 *      Intake:
 *          Left-trigger               -> Outtake
 *          Right-trigger              -> Intake
 *
 *      OutTake:
 *
 *
 *      Drone:
 *          Left-bumper                -> Launch drone
 *
 *
 *
 *
 *   User 2: (Richie & Khallie)
 *
 *      Lift:
 *          Dpad-up                    -> High pose
 *          Dpad-down                  -> Start pose
 *          Dpad-right                 -> Low pose
 *          Dpad-left                  -> Mid pose
 *
 *     outtake:
 *          Left-trigger               -> PICKUP
 *          Right-trigger              -> PRERELEASE
 *          Right-Bumper               -> RELEASE
 *
 *      Drone:
 *          left-bumper                -> open
 *
 *      Hanging:
 *          y-button                   -> hanging goes up
 *          a-button                   -> hanging goes down
 *          b-button                   -> STOP
 *
 *
 *
 *
 * @see CenterStageRobot
 */

@TeleOp(name = "CS TeleOp LIO", group = "Main")
public class CSTeleopLIO extends CSTeleopRobotLIO {

    private double currentTime = 0; // keep track of current time
    private double speedMultiplier = 0.7;
    //these are based on LiftTest
    private static final double HIGH = 26d;
    private static final double MID = 8d;
    private static final double LOW = 10d;
    private boolean liftdown = true;
    private boolean intakeOn = false;



//    private boolean armMid = true;

    //private boolean coneloaded = false;
    private Pose2d poseEstimate;

    //RevBlinkinLedDriver blinkinLedDriver;
    //RevBlinkinLedDriver.BlinkinPattern pattern;
    //private ElapsedTime timeSinceIntakeButton = new ElapsedTime();

    @Override
    public void init() {
        drive = new CSBaseLIO(hardwareMap, true);
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        super.init();
        drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);

        drive.setPoseEstimate(PoseStorage.currentPose);  //Added to start of TeleOp with how Auto ended
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        poseEstimate = drive.getPoseEstimate();

        //---------------------------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier
                )
        );

        //this changes the speed the robot moves at
        if (getEnhancedGamepad1().isLeftBumperJustPressed()) {
            speedMultiplier = 0.7;
        }
        if (getEnhancedGamepad1().isRightBumperJustPressed()) {
            speedMultiplier = 1.0;
        }

        if (getEnhancedGamepad1().isxJustPressed()) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.OUTTAKE);
            intakeOn = true;
        }

        //Intake
        //spins the intake to intake a pixel
        if (getEnhancedGamepad1().getLeft_trigger() > 0) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.OUTTAKE);
            intakeOn = true;
        }
        //spins the intake to outtake a pixel
        if (getEnhancedGamepad1().getRight_trigger() > 0) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
            intakeOn = true;
        }
        //stops spining the intake
        if (getEnhancedGamepad1().isStart()) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
            intakeOn = false;
        }

        //This stops running the intake when the lift is up
        if (intakeOn && !liftdown) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
        }

        //---------------------------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 2

        //Drone
       telemetry.addData("Drone State: ", drive.robot.getDroneSubsystem().getStateMachine().getState());
        if (getEnhancedGamepad2().isLeftBumperJustPressed()) {
            drive.robot.getDroneSubsystem().getStateMachine().updateState(DroneStateMachine.State.OPEN);
        }

        //Lift
        //brings the lift down to the starting pose`
        if (getEnhancedGamepad2().isDpadDownJustPressed()) {
            double lastSetPoint = drive.robot.getLiftSubsystem().getDesiredSetpoint();
            telemetry.addData("Lift State: ", lastSetPoint);
            drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
            drive.robot.getLiftSubsystem().retract();
            liftdown = true;
        }

        //Brings the lift down to the LOW pose when b is pressed
        if (getEnhancedGamepad2().isDpadRightJustPressed()) {  //&&arm mid
            drive.robot.getLiftSubsystem().extend(LOW);
            liftdown = false;
        }
        //Brings the lift down to the MID pose when b is pressed
        if (getEnhancedGamepad2().isDpadLeftJustPressed()) {  //&&arm mid
            drive.robot.getLiftSubsystem().extend(MID);
            liftdown = false;
        }
        //Brings the lift down to the LOW pose when b is pressed
        if (getEnhancedGamepad2().isDpadUpJustPressed()) {  //&&arm mid
            drive.robot.getLiftSubsystem().extend(HIGH);
            liftdown = false;
        }

        //Hang
        if(getEnhancedGamepad2().isyLast()){
            drive.robot.getHangSubsystem().getStateMachine().updateState(HangStateMachine.State.UP);
        }

        if(getEnhancedGamepad2().isaLast()){
            drive.robot.getHangSubsystem().getStateMachine().updateState(HangStateMachine.State.DOWN);
        }

        if(getEnhancedGamepad2().isbLast()){
            drive.robot.getHangSubsystem().getStateMachine().updateState(HangStateMachine.State.IDLE);
        }


        //Outtake
        //This only allows moving the Outtake only when the Lift has moved up and is no longer in Ground or Intake Position
        //This also brings the lift down to the ground state when Outtake is Released
       if (!liftdown) {
            if (getEnhancedGamepad2().getRight_trigger() > 0) {
                drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.RELEASE);
            }
            if (getEnhancedGamepad2().getLeft_trigger() > 0) {
                drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
            }
            if (liftdown) {
                drive.robot.getOuttakeSubsystem().getStateMachine().updateState(OuttakeStateMachine.State.PICKUP);
            }

        }

        telemetry.addData("Lift State: ", drive.robot.getLiftSubsystem().getStateMachine().getState());
        telemetry.addData("Lift SetPoint: ", drive.robot.getLiftSubsystem().getDesiredSetpoint());
        telemetry.addData("Outtake State: ", drive.robot.getOuttakeSubsystem().getStateMachine().getState());

            updateTelemetry(telemetry);
            currentTime = getRuntime();
        }
}

