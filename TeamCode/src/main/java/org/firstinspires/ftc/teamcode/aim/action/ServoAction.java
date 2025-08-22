package org.firstinspires.ftc.teamcode.aim.action;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoAction extends Action {
    private ParallelAction topAct;

    public ServoAction(String name, Servo servo, double pos, long milliSecs) {
        super(name);
        Action posAct = new Action("servoSetPos") {
            @Override
            public boolean run() {
                if (isStarted()) {
                    return true;
                }
                markStarted();
                servo.setPosition(pos);
                return true;
            }
        };

        this.topAct = new ParallelAction("setServoPosPara");
        topAct.addAction(posAct);
        topAct.addAction(new SleepAction("sleep", milliSecs));
    }

    @Override
    public boolean run() {
        return topAct.run();
    }
}
