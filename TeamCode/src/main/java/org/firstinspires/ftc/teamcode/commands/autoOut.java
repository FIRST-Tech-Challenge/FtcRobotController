package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

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
        this.robot.intake.toBasePos();
        this.robot.outtake.prepOuttake();
        time = System.currentTimeMillis();
        state = 1;
    }

    @Override
    public void update() {
        if (state == 1) { // wait for lift state to be != 0
            if (System.currentTimeMillis() - time > 100) {
                state = 2;
            }
        } else if (state == 2) {//wait for lift reach state 0
            if (robot.outtake.liftState == 0 && robot.outtake.swingState == 0) { //gamepad2.a done,
                time = System.currentTimeMillis();
                state = 3;
            }
        } else if (state == 3) {
            if (System.currentTimeMillis() - time > 1000) { // wait for dumpServo reach carryPos
                robot.outtake.dropPixelPos();    //dump pixel
                state = 4;
                time = System.currentTimeMillis();
            }
        } else if (state == 4) { // dump pixel, wait for 1 s
            if (System.currentTimeMillis() - time > 1000) {
                robot.outtake.toIntakePos(); // back to intake pos
                state = 5;
            }
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        if (this.state == 5 && robot.outtake.swingState == 0 && robot.outtake.liftState == 0) {
            state = 0;
            return (true);
        } else {
            return (false);
        }
    }
}
