package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.ProgrammingBoard;
import org.firstinspires.ftc.teamcode.mechanism.Traction;

@Autonomous
public class RedRight extends OpMode {

    enum State {
        RIGHT,
        JOLT,
        BACK,
        STOP,
        FINISH
    }

    State state = State.RIGHT;
    ProgrammingBoard Board = new ProgrammingBoard();
    Traction DriveTrain = new Traction();
    @Override
    public void init() {
        Board.init(hardwareMap);
    }
    @Override
    public void start(){
        resetRuntime();
        State state = State.RIGHT;

    }

    @Override
    public void loop(){
        telemetry.addData("Time", getRuntime());
        telemetry.update();


        switch (state) {

            case RIGHT:
                DriveTrain.controllerDrive(0, .5, 0);
                state = State.JOLT;
                break;

            case JOLT:
                if (getRuntime() >= 2.25) {
                    DriveTrain.controllerDrive(.5, 0, 0);
                    state = State.BACK;
                    Board.setIntakePower(-1);
                }
                break;

            case BACK:
                if (getRuntime() >= 2.5) {
                    DriveTrain.controllerDrive(-.5, 0, 0);
                    state = State.STOP;
                }
                break;
            case STOP:
                if (getRuntime() >= 2.75) {
                    DriveTrain.controllerDrive(0, 0, 0);
                    state = State.FINISH;
                }

            case FINISH:
                if (getRuntime() >= 10) {
                    DriveTrain.controllerDrive(0, 0, 0);
                    Board.setIntakePower(0);
                }
                break;


        }
        }
}
