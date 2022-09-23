
package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IntakeSystem;
import org.firstinspires.ftc.teamcode.components.TensorFlow;
import org.firstinspires.ftc.teamcode.components.TurnTableSystem;
import org.firstinspires.ftc.teamcode.helpers.Constants;
import org.firstinspires.ftc.teamcode.helpers.GameState;
import org.firstinspires.ftc.teamcode.helpers.RouteState;
import org.firstinspires.ftc.teamcode.helpers.TeamState;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

public abstract class AutonomousOpMode extends BaseOpMode {


    private double elevatorLevel = -3333333;
    private boolean isRBorBT;
    private double baseTime;
    private boolean primary_scan;
    private boolean secondary_scan;

    private GameState currentGameState;
    public RouteState routeState;
    protected TeamState teamState;
    private TensorFlow tensorFlow;

    private static final double driveSpeed = 0.5;
    private static final double rotateSpeed = 0.25;

    private static final int level = 2;

    public void init(TeamState teamState, RouteState routeState) {
        isRBorBT = ((teamState == TeamState.BLUE && routeState == RouteState.TOP) || (teamState == TeamState.RED && routeState == RouteState.BOTTOM));
        super.init();
        this.teamState = teamState;
        this.routeState = routeState;
        driveSystem.initMotors();
        //vuforia = new Vuforia(hardwareMap.get(WebcamName.class, "Webcam 1"),0 );
        tensorFlow = new TensorFlow(hardwareMap);
        //vuforia.activate();
        tensorFlow.activate();
        armSystem = new ArmSystem(hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR), hardwareMap.get(AnalogInput.class, "p"));
        armSystem.initMotors();
        intakeSystem = new IntakeSystem(hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR1), hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR2));
        intakeSystem.initMotors();
        turnTableSystem = new TurnTableSystem(hardwareMap.get(DcMotor.class, Constants.ROTATOR_MOTOR));
        armSystem.moveToPosition(ArmSystem.LEVEL_CAROUSEL);
        armSystem.stop();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        primary_scan = tensorFlow.getInference().size() > 0;
        telemetry.addData("DUCK?", tensorFlow.getInference().size() > 0);
    }

    @Override
    public void loop() {
        telemetry.addData("GameState", currentGameState);
        telemetry.addData("elevatorLevel", elevatorLevel);
        telemetry.addData("voltage", armSystem.getSensorAsAnalogInput0());
        telemetry.update();

        armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        switch (currentGameState) {
            case SCAN_INITIAL:
                if (primary_scan) {
                    elevatorLevel = ArmSystem.LEVEL_CAROUSEL;
                    newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_ONE);
                } else {
                    newGameState(GameState.SCAN_SECONDARY);
                }
                break;
            case SCAN_SECONDARY:
                if (driveSystem.driveToPosition((int) (Constants.tileWidth * Constants.mmPerInch * (2/3)), isRBorBT ? DriveSystem.Direction.FORWARD : DriveSystem.Direction.BACKWARD, driveSpeed)) {
                    if (secondary_scan) {
                        elevatorLevel = isRBorBT ? ArmSystem.LEVEL_BOTTOM : ArmSystem.LEVEL_TOP;
                        newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_ONE);
                    } else {
                        elevatorLevel = isRBorBT ? ArmSystem.LEVEL_TOP : ArmSystem.LEVEL_BOTTOM;
                    }
                    newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_ONE);
                } else {
                    secondary_scan = tensorFlow.getInference().size() > 0;
                }
                break;
            case DRIVE_TO_ALLIANCE_HUB_ONE:
                if (driveSystem.driveToPosition((int) ((Constants.tileWidth - 5) * Constants.mmPerInch), !isRBorBT ? DriveSystem.Direction.BACKWARD : DriveSystem.Direction.FORWARD, driveSpeed)) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.DRIVE_TO_ALLIANCE_HUB_TWO);
                }
                break;
            case DRIVE_TO_ALLIANCE_HUB_TWO:
                if (driveSystem.driveToPosition((int) (0.75 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.LEFT, driveSpeed * 0.25)) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.ROTATE_TURNTABLE);
                }
                break;
            case ROTATE_TURNTABLE:
                while (turnTableSystem.getPosition() != TurnTableSystem.LEVEL_90) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    turnTableSystem.moveToPosition(TurnTableSystem.LEVEL_90);
                }
                baseTime = 0;
                newGameState(GameState.PLACE_CUBE);
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
                while (armSystem.notTooHigh()) {
                    armSystem.move_up();
                }
                if (baseTime == 0) {
                    baseTime = elapsedTime.seconds();
                }
                if (elapsedTime.seconds() < baseTime + 2.0) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    intakeSystem.spit_out();
                } else {
                    intakeSystem.setPower(0);
                    while (turnTableSystem.getPosition() != TurnTableSystem.LEVEL_0) {
                        armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        armSystem.stop();
                        turnTableSystem.moveToPosition(TurnTableSystem.LEVEL_0);
                    }
                    while (armSystem.getSensorAsAnalogInput0() < 1) {
                        armSystem.move_down();
                    }
                    baseTime = 0;
                    newGameState(routeState == RouteState.TOP ? GameState.PARK_IN_TOP_WAREHOUSE_ONE : GameState.DRIVE_TO_CAROUSEL_ONE);
                }
                break;
            case DRIVE_TO_CAROUSEL_ONE:
                if (teamState == TeamState.BLUE ?
                        driveSystem.turn(78, rotateSpeed) :
                        driveSystem.driveToPosition((int) (0.75 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.RIGHT, driveSpeed * 0.25)) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.DRIVE_TO_CAROUSEL_TWO);
                }
                break;

            case DRIVE_TO_CAROUSEL_TWO:
                if (driveSystem.driveToPosition(teamState == TeamState.BLUE ? (int) (2.25 * Constants.tileWidth * Constants.mmPerInch) : (int) ((Constants.tileWidth - 10) * Constants.mmPerInch), teamState == TeamState.BLUE ? DriveSystem.Direction.LEFT : DriveSystem.Direction.BACKWARD, driveSpeed)) {
                    driveSystem.setMotorPower(0);
                    newGameState(GameState.DRIVE_TO_CAROUSEL_THREE);
                }
                break;
            case DRIVE_TO_CAROUSEL_THREE:
                if (driveSystem.driveToPosition(teamState == TeamState.BLUE ? (int) (6.25 * Constants.mmPerInch) : (int) (16.25 * Constants.mmPerInch), DriveSystem.Direction.FORWARD, driveSpeed * 0.5)) {
                    driveSystem.setMotorPower(0);
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.SPIN_CAROUSEL);
                    baseTime = 0;
                }
                break;
            case SPIN_CAROUSEL:
                if (baseTime == 0) {
                    baseTime = elapsedTime.seconds();
                }
                armSystem.stop();
                if (elapsedTime.seconds() < baseTime + 6.0) {
                    intakeSystem.Carousel(teamState);
                } else {
                    intakeSystem.setPower(0);
                    newGameState(GameState.PARK_IN_BOTTOM_WAREHOUSE);
                }
                break;

            case PARK_IN_BOTTOM_WAREHOUSE:
                if (driveSystem.driveToPosition((int) (0.5 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.BACKWARD, driveSpeed)) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.COMPLETE);
                }
                break;

            case PARK_IN_TOP_WAREHOUSE_ONE:
                if (driveSystem.driveToPosition((int) (0.5 * Constants.tileWidth * Constants.mmPerInch), teamState == TeamState.BLUE ? DriveSystem.Direction.LEFT : DriveSystem.Direction.RIGHT, driveSpeed)) {
                    newGameState(GameState.PARK_IN_TOP_WAREHOUSE_TWO);
                }
                break;

            case PARK_IN_TOP_WAREHOUSE_TWO:
                if (driveSystem.driveToPosition((int) (3 * Constants.tileWidth * Constants.mmPerInch), teamState == TeamState.BLUE ? DriveSystem.Direction.BACKWARD : DriveSystem.Direction.FORWARD, driveSpeed)) {
                newGameState(GameState.COMPLETE);
            }
            break;

            case COMPLETE:
                armSystem.moveToPosition(ArmSystem.LEVEL_TOP);
                armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                driveSystem.setMotorPower(0);
                turnTableSystem.moveToPosition(teamState == TeamState.BLUE ? -TurnTableSystem.LEVEL_90 : TurnTableSystem.LEVEL_90);
                armSystem.stop();
                telemetry.speak("among us among us among us");
                stop();
                break;
        }
    }

    @Override
    public void stop() {
        super.stop();
        tensorFlow.shutdown();
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