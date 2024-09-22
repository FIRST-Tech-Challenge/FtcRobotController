package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoTimedF4", group = "Furious Frogs")
//@Disabled
public class TimeBasedAutoF4 extends TimeBasedAutoBase {
    public void goToBackStage() {
        // move forward till center line
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            move(0, 1, 0);
        }
        stopChassis();

        //rotate
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.44) {
            move(0, 0, -1);
        }
        stopChassis();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.8) {
            move(0, 1, 0);
        }
        stopChassis();

        //strafe a little
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.2) {
            move(-1, 0, 0);
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

        while (opModeIsActive() && (runtime.seconds() < 1.6)) {
            move(1, 0, 0);
        }
        stopChassis();


        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < .45)) {
            move(0, 1, 0);
        }
        stopChassis();


    }

}
