package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="test")
public class AutoTest extends MasterAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        robot.setInitialAngle();

        /*
        wristServo.setPosition(0.0);
        runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_3, 0.2);
        runMotorToPosition(elbowMotor, ELBOW_LEVEL_3, 0.5);
        while(shoulderMotor.isBusy() || elbowMotor.isBusy()) {
            telemetry.addLine("pathli");
        }
        */
        pivot(90, 0.6);


        //moveInches(15, 0.8);
        //pivot(-180, 0.2);
        //moveInches(15, 0.2);
        /*telemetry.addData("target", targetAngle);
        telemetry.update();
        moveInches(15, 0.5);
        pivot(180, 0.5);
        moveInches(15, 0.5);
        pivot(-90, 0.5);
        moveInches(15, 0.5);
        pivot(0, 0.5);
        moveInches(15, 0.5);*/

    }
}
