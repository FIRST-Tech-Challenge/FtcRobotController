package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IntakeSystem;
import org.firstinspires.ftc.teamcode.components.TensorFlow;
import org.firstinspires.ftc.teamcode.components.TurnTableSystem;
import org.firstinspires.ftc.teamcode.helpers.Constants;
import org.firstinspires.ftc.teamcode.helpers.GameState;
import org.firstinspires.ftc.teamcode.helpers.TeamState;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@Autonomous(name = "RED, BOTTOM", group = "Autonomous")
public class AutonomousOpModeRedBottom extends BaseOpMode {

    private double elevatorLevel = -3333333;
    private double baseTime;

    private boolean primary_scan = false;
    private boolean secondary_scan = false;

    private GameState currentGameState;
    private TensorFlow tensorFlow;

    private static final double driveSpeed = 0.5;
    private static final double rotateSpeed = 0.25;

    @Override
    public void init() {
        super.init();
        driveSystem.initMotors();
        //tensorFlow = new TensorFlow(hardwareMap);
        //tensorFlow.activate();
        armSystem = new ArmSystem(hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR), hardwareMap.get(AnalogInput.class, "p"));
        armSystem.initMotors();
        intakeSystem = new IntakeSystem(hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR1), hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR2));
        intakeSystem.initMotors();
        turnTableSystem = new TurnTableSystem(hardwareMap.get(DcMotor.class, Constants.ROTATOR_MOTOR));
        boolean directionNeedsToGo = armSystem.getSensorAsAnalogInput0() < ArmSystem.LEVEL_CAROUSEL;
        armSystem.moveToCarousel(directionNeedsToGo);
        armSystem.stop();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        primary_scan = true;
        //telemetry.addData("DUCK?", tensorFlow.seesDuck());
    }

    @Override
    public void start() {
        super.start();
        newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_ONE_PRIMARY);
    }

    @Override
    public void loop() {
        telemetry.addData("GameState", currentGameState);
        telemetry.addData("elevatorLevel", elevatorLevel);
        telemetry.addData("voltage", armSystem.getSensorAsAnalogInput0());
        telemetry.addData("vel", imuSystem.imu.getVelocity());
        telemetry.update();

        armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSystem.stop();

        switch (currentGameState) {
            case SCAN_INITIAL:
                if (primary_scan) {
                    elevatorLevel = ArmSystem.LEVEL_CAROUSEL;
                    newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_ONE_PRIMARY);
                } else {
                    newGameState(GameState.SCAN_SECONDARY);
                }
                break;
            case SCAN_SECONDARY:
                if (driveSystem.driveToPosition((int) (Constants.tileWidth * Constants.mmPerInch * (2/3)), DriveSystem.Direction.FORWARD, driveSpeed)) {
                    if (secondary_scan) {
                        elevatorLevel = ArmSystem.LEVEL_BOTTOM;
                    } else {
                        elevatorLevel = ArmSystem.LEVEL_TOP;
                    }
                    newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_ONE_SECONDARY);
                } else {
                    //secondary_scan = tensorFlow.getInference().size() > 0;
                }
                break;
            case DRIVE_TO_ALLIANCE_HUB_ONE_PRIMARY:
                if (driveSystem.driveToPosition(((int) (0.9 * (Constants.tileWidth - 5) * Constants.mmPerInch)), DriveSystem.Direction.FORWARD, driveSpeed * 0.5)) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_TWO);
                }
                break;
            case DRIVE_TO_ALLIANCE_HUB_TWO:
                if (driveSystem.driveToPosition((int) (0.8 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.LEFT, driveSpeed * 0.5)) {
                    //armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    telemetry.addLine("hello world");
                    newGameState(GameState.ROTATE_TURNTABLE);
                }
                break;
            case ROTATE_TURNTABLE:
                //armSystem.moveToPosition(ArmSystem.LEVEL_TOP);
                telemetry.addLine("we're here");
                armSystem.stop();
                if (turnTableSystem.moveToPosition(TurnTableSystem.LEVEL_90)){
                    telemetry.addLine("tf?");
                    newGameState(GameState.PLACE_CUBE);
                }
                break;
            case PLACE_CUBE:
                if (elevatorLevel == -3333333){
                    double giveUpHope = Math.random();
                    if (giveUpHope < 0.33) {
                        elevatorLevel = ArmSystem.LEVEL_BOTTOM;
                    }
                    if (giveUpHope < 0.66) {
                        elevatorLevel =  ArmSystem.LEVEL_CAROUSEL;
                    }
                    else {
                        elevatorLevel = ArmSystem.LEVEL_TOP;
                    }
                }
                if (baseTime == 0) {
                    baseTime = elapsedTime.seconds();
                }
                if (elapsedTime.seconds() < baseTime + 2.0) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    intakeSystem.spit_out(0.35); // requries testings
                } else {
                    intakeSystem.setPower(0);
                    if (turnTableSystem.moveToPosition(TurnTableSystem.LEVEL_0)){
                        armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        armSystem.stop();
                        baseTime = 0;
                        newGameState(GameState.DRIVE_TO_CAROUSEL_ONE);
                    }
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    baseTime = 0;
                    newGameState(GameState.DRIVE_TO_CAROUSEL_ONE);
                }
                break;
            case DRIVE_TO_CAROUSEL_ONE:
                if (driveSystem.turn(70, rotateSpeed)) { // [TODO - degrees needs to change, AC]
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.DRIVE_TO_CAROUSEL_TWO);
                }
                break;

            case DRIVE_TO_CAROUSEL_TWO:
                if (driveSystem.driveToPosition((int) (2.25 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.RIGHT, driveSpeed)) {
                    driveSystem.setMotorPower(0);
                    //armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.moveToCarousel(false);
                    telemetry.addLine(Double.toString(armSystem.getSensorAsAnalogInput0()));
                    armSystem.stop();
                    armSystem.moveToCarousel(false);
                    armSystem.stop();
                    newGameState(GameState.DRIVE_TO_CAROUSEL_THREE);
                }
                break;
            case DRIVE_TO_CAROUSEL_THREE:
                if (driveSystem.driveToPosition((int) (0.8 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.FORWARD, driveSpeed * 0.75)) {
                    driveSystem.setMotorPower(0);
                    baseTime = 0;
                    newGameState(GameState.SPIN_CAROUSEL);
                }
                break;
            case SPIN_CAROUSEL:
                if (baseTime == 0) {
                    baseTime = elapsedTime.seconds();
                }
                armSystem.stop();
                if (elapsedTime.seconds() < baseTime + 6.0) {
                    intakeSystem.Carousel(TeamState.RED);
                } else {
                    intakeSystem.setPower(0);
                    newGameState(GameState.PARK_IN_BOTTOM_WAREHOUSE);
                }
                break;

            case PARK_IN_BOTTOM_WAREHOUSE:
                if (driveSystem.driveToPosition((int) (0.65 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.BACKWARD, driveSpeed)) { // [TODO - distance should increase, AC}
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.moveToMaxHeight();
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    driveSystem.setMotorPower(0);
                    armSystem.stop();
                    newGameState(GameState.COMPLETE);
                }
                break;

            case COMPLETE:
                stop();
                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
        //tensorFlow.shutdown();
    }

    /**
     * Updates the state of the system and updates RoadRunner trajectory
     *
     * @param newGameState to switch to
     */
    protected void newGameState(GameState newGameState) {
        currentGameState = newGameState;
    }

}