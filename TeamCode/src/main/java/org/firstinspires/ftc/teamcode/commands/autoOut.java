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
    public void start() {
        this.robot.intake.setPower(0);
        this.robot.intake.toBasePos();
        this.robot.outtake.prepOuttake();
        state = 1;
    }

    @Override
    public void update() {
        if (state == 1) {//wait for lift reach state 0
            if (robot.outtake.liftState == 0) {
                time = System.currentTimeMillis();
                robot.outtake.dropPixelPos(); //dump pixel
                state = 2;
            }
        } else if (state == 2) { // dump pixel, wait for 1 s
            if (System.currentTimeMillis() - time > 3000) {
                robot.outtake.toIntakePos(); // back to intake pos
                state = 3;
            } else {
                robot.outtake.dropPixelPos();
            }
        } else if (state == 3) { // wait for swingState == 0
            if (robot.outtake.swingState == 0) {
                state = 4;
            }
        }

    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        if (this.state == 4 && this.robot.outtake.liftState == 0) {
            state = 0;
            return (true);
        } else {
            return (false);
        }
    }
}
