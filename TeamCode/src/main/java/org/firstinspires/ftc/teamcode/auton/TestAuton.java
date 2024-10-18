package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous(name = "Pedro Auton")
public class TestAuton extends OpMode {

    private enum Path {
        PATH_1,
        PATH_2
    }

    Follower follower;

    Path currentPath;
    @Override
    public void init() {

        follower = new Follower(hardwareMap);
        currentPath = Path.PATH_1;

    }

    @Override
    public void loop() {
        switch(currentPath) {
            case PATH_1:

                //insert code here
                break;
            case PATH_2:
                // insert code here
                break;
        }

    }
}
