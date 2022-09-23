package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IntakeSystem;
import org.firstinspires.ftc.teamcode.components.TensorFlow;
import org.firstinspires.ftc.teamcode.components.TurnTableSystem;
import org.firstinspires.ftc.teamcode.helpers.Constants;
import org.firstinspires.ftc.teamcode.helpers.GameState;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.Map;

@Autonomous(name = "RED, TOP", group = "Autonomous")
public class AutonomousOpModeRedTop extends BaseOpMode {


    private double elevatorLevel = -3333333;
    private double baseTime;

    private boolean primary_scan;
    private boolean secondary_scan;

    private GameState currentGameState;
    //private Vuforia vuforia;
    private TensorFlow tensorFlow;

    private static final double driveSpeed = 0.5;
    private static final double rotateSpeed = 0.25;

    @Override
    public void init() {
        super.init();
        driveSystem.initMotors();
        //vuforia = new Vuforia(hardwareMap.get(WebcamName.class, "Webcam 1"),0 );
        //tensorFlow = new TensorFlow(hardwareMap);
        //vuforia.activate();
        //tensorFlow.activate();
        armSystem = new ArmSystem(hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR), hardwareMap.get(AnalogInput.class, "p"));
        armSystem.initMotors();
        intakeSystem = new IntakeSystem(hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR1), hardwareMap.get(DcMotor.class, Constants.INTAKE_MOTOR2));
        intakeSystem.initMotors();
        turnTableSystem = new TurnTableSystem(hardwareMap.get(DcMotor.class, Constants.ROTATOR_MOTOR));
        currentGameState = GameState.SCAN_INITIAL;
    }

    @Override
    public void init_loop() {
        super.init_loop();
        primary_scan = true;
        //primary_scan = tensorFlow.getInference().size() > 0;
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
        for (Map.Entry<DriveSystem.MotorNames, DcMotor> motor : driveSystem.motors.entrySet()) {
            Log.d("CURRENT POSITION", String.valueOf(motor.getValue().getCurrentPosition()));
            Log.d("CURRENT POSITION", String.valueOf(motor.getValue().getTargetPosition()));
        }

        telemetry.update();

        armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        switch (currentGameState) {
            case DRIVE_TO_ALLIANCE_HUB_ONE_PRIMARY:
                if (driveSystem.driveToPosition((int) (3.5 * Constants.tileWidth * Constants.mmPerInch), DriveSystem.Direction.FORWARD, driveSpeed)) {
                    armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armSystem.stop();
                    newGameState(GameState.COMPLETE);
                }
                break;
            case COMPLETE:
                armSystem.getElevatorMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                driveSystem.setMotorPower(0);
                armSystem.stop();
                telemetry.speak("among us among us among us");
                stop();
                break;
        }    }

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