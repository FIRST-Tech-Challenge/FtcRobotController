package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

//autoOut needs to do 3 outtake actions:
// 1:             robot.intake.setPower(0);
//                robot.intake.toBasePos();
//                robot.outtake.prepOuttake();
// 2:             robot.outtake.dropPixelPos();
// 3:             robot.outtake.toIntakePos();
public class autoOutDump implements Command {

    CrabRobot robot;
    //double duration;
    NanoClock clock;
    long time = System.currentTimeMillis();
    int state = 0;

    public autoOutDump(CrabRobot robot) {
        this.robot= robot;
        clock = NanoClock.system();
    }

    @Override
    public void start() { // gamepad2.a
        this.robot.intake.setPower(0);
        this.robot.intake.toBasePosYield();
        robot.outtake.dropPixelPosAuto();
        time = System.currentTimeMillis();
        state = 1;
        Log.v("autoOut", "dropPixelPosAuto()");
    }

    @Override
    public void update() {
        if (state == 1) { // drop pixel time
            if (System.currentTimeMillis() - time > 1000) {
                robot.outtake.dumperToTravelPosAuto();
                state = 2;
                Log.v("autoOutDump", "-> dumperToTravelPosAuto()");
            }
        } else if (state == 2) { // dump pixel, wait for 1 s
            if (System.currentTimeMillis() - time > 100) { // dumper reset time
                robot.outtake.toIntakePosAuto(); // back to intake pos
                state = 3;
                time = System.currentTimeMillis();
                Log.v("autoOutDump", "-> toIntakePosAuto()");
            }
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        if (this.state == 3
                && robot.outtake.swingState == 0
                && robot.outtake.liftState == 0
                && (System.currentTimeMillis() - time > 400)) {
            state = 0;
            return (true);
        } else {
            return (false);
        }
    }
}
