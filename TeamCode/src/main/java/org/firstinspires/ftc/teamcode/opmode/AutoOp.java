package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.ProgrammingBoard;
import org.firstinspires.ftc.teamcode.mechanism.Traction;
import org.opencv.objdetect.Board;

@Autonomous
public class AutoOp extends OpMode {

    double lastTime;
    enum State {
        STRAFE,
        STOP
    }

    State state = State.STRAFE;
    ProgrammingBoard Board = new ProgrammingBoard();
    Traction DriveTrain = new Traction();
    @Override
    public void init() {
        Board.init(hardwareMap);
    }
    @Override
    public void start(){
        resetRuntime();
        lastTime = getRuntime();
        State state = State.STRAFE;

    }

    @Override
    public void loop(){
        telemetry.addData("Time", lastTime);
        DriveTrain.controllerDrive(0.5, 0, 0);

        switch (state) {
            case STRAFE:
                if (getRuntime() >= 2) {
                    state = State.STOP;
                    DriveTrain.controllerDrive(0,.5,0);
                }
                break;
            case STOP:
                if (getRuntime() >= 4) {
                    DriveTrain.controllerDrive(0,0,0);
                }
                break;
        }







        telemetry.update();

    }
}
