package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autonomous")
public class Auto extends LinearOpMode {
    Robot robot = new Robot();
    enum autoSteps {
        ORIGIN,
        GOINGTOPOSITION1,
        ATPOSITION1,
        GOINGTOPOSITION2,
        ATPOSITION2,
        GOINGTOPOSIITION3,
        ATPOSITION3
    }
    autoSteps currentStep = autoSteps.ORIGIN;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();




        while (opModeIsActive()) {

            robot.db.update();
            switch(currentStep){
                case ORIGIN:
                    //do stuff ie, move arm
                    robot.db.setTargetPosition(robot.db.ac.test);
                    currentStep = autoSteps.GOINGTOPOSITION1;
                    break;
                case GOINGTOPOSITION1:
                    if (robot.db.isAtTargetPosition()) {
                        currentStep = autoSteps.ATPOSITION1;
                    }
                    break;
                case ATPOSITION1:
                    //do stuff ie, move arm
                    robot.db.setTargetPosition(robot.db.ac.test2);
                    break;
                case GOINGTOPOSITION2:
                    if (robot.db.isAtTargetPosition()) {
                        currentStep = autoSteps.ATPOSITION2;
                    }
                    break;
                case ATPOSITION2:
                    //do stuff ie, move arm
                    stop();
                    break;
            }
        }
    }
}