package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoTimedF2", group = "Furious Frogs")
//@Disabled
public class TimeBasedAutoF2 extends TimeBasedAutoBase {
    public void goToBackStage() {
        //go straight
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2.9) {
            move(0, 1, 0);
        }
        stopChassis();

        //rotate
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.49) {
            move(0, 0, -1);
        }
        stopChassis();

        //go straight through the door
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.78)) {
            move(0, 1, 0);
        }
        stopChassis();
        runtime.reset();

        //strafe
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            move(1, 0, 0);
        }
        stopChassis();
        runtime.reset();
    }


    public void park() {
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < .7)) {
            move(0, -1, 0);
        }
        stopChassis();


        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            move(-1, 0, 0);
        }
        stopChassis();


        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < .5)) {
            move(0, 1, 0);
        }
        stopChassis();


    }

}
