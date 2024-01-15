package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import android.util.Log;

//autoOut needs to do 3 outtake actions:
// 1:             robot.intake.setPower(0);
//                robot.intake.toBasePos();
//                robot.outtake.prepOuttake();
// 2:             robot.outtake.dropPixelPos();
// 3:             robot.outtake.toIntakePos();
public class autoOut implements Command {

    CrabRobot robot;
    //double duration;
    NanoClock clock;
    long time = System.currentTimeMillis();
    int state = 0;

    public autoOut(CrabRobot robot) {
        this.robot= robot;
        clock = NanoClock.system();
    }

    @Override
    public void start() { // gamepad2.a
        this.robot.intake.setPower(0);
        this.robot.intake.toBasePosYield();
        this.robot.outtake.prepOuttakeAuto();
        time = System.currentTimeMillis();
        state = 1;
        Log.v("autoOut", "start()");
    }

    @Override
    public void update() {
        if (state == 1) { // wait for lift state to be != 0
            if (System.currentTimeMillis() - time > 100) {
                state = 2;
                Log.v("autoOut", "state 1 -> 2");
            }
        } else if (state == 2) {//wait for lift reach state 0
            if (robot.outtake.liftState == 0 && robot.outtake.swingState == 0) { //gamepad2.a done,
                time = System.currentTimeMillis();
                state = 3;
                Log.v("autoOut", "state 2 -> 3");
            }
        } else if (state == 3) {
            if (System.currentTimeMillis() - time > 400) { // wait for dumpServo reach carryPos
                //robot.outtake.dropPixelPos();    //dump pixel
                robot.outtake.dropPixelPosAuto();
                state = 4;
                time = System.currentTimeMillis();
                Log.v("autoOut", "state 3 -> 4");
            }
        } else if (state == 4) { // dump pixel, wait for 1 s
            if (System.currentTimeMillis() - time > 800) { // drop pixel time
                state = 5;
                //robot.outtake.dumperToIntakePosAuto();
                robot.outtake.dumperToTravelPosAuto();
                Log.v("autoOut", "state 4 -> 5");
            }
        } else if (state == 5) {
            if (System.currentTimeMillis() - time > 100) { // dumper reset time
                robot.outtake.toIntakePosAuto(); // back to intake pos
                state = 6;
                time = System.currentTimeMillis();
                Log.v("autoOut", "state 5 -> ");
            }
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        if (this.state == 6 && robot.outtake.swingState == 0 && robot.outtake.liftState == 0) {
            state = 0;
            return (true);
        } else {
            return (false);
        }
    }
}
