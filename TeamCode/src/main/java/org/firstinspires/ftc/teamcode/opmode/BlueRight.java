package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.ProgrammingBoard;
import org.firstinspires.ftc.teamcode.mechanism.Traction;

@Autonomous
public class BlueRight extends OpMode {

    enum State {
        INITIAL,
        STRAFE1,
        JOLT,
        BACK,
        STOP,
        FINISH
    }

    State state = State.INITIAL;
    ProgrammingBoard Board = new ProgrammingBoard();
    Traction DriveTrain = new Traction();

    @Override
    public void init() {
        Board.init(hardwareMap);
    }

    @Override
    public void start() {
        resetRuntime();
        State state = State.INITIAL;
    }

    @Override
    public void loop() {
        telemetry.addData("Time", getRuntime());

        switch (state) {
            case INITIAL:
                DriveTrain.controllerDrive(0.5, 0, 0);
                state = State.STRAFE1;
                break;

            case STRAFE1:
                if (getRuntime() >= .1) {
                    state = State.JOLT;
                    DriveTrain.controllerDrive(0, -.5, 0);
                }
                break;
            case JOLT:
                if (getRuntime() >= 4) {
                    DriveTrain.controllerDrive(.5, 0, 0);
                    state = State.BACK;
                    Board.setIntakePower(-1);
                }
                break;
            case BACK:
                if (getRuntime() >= 4.25) {
                    DriveTrain.controllerDrive(-.5, 0, 0);
                    state = State.STOP;
                }
                break;
            case STOP:
                if (getRuntime() >= 4.5) {
                    DriveTrain.controllerDrive(0,0,0);
                    state = State.FINISH;
                }
                telemetry.update();

            case FINISH:
                if (getRuntime() >= 10) {
                    DriveTrain.controllerDrive(0, 0, 0);
                    Board.setIntakePower(0);
                }
                break;


        }
    }
}
