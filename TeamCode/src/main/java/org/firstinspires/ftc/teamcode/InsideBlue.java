package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Inside Blue")
public class InsideBlue extends LinearOpMode {

    DcMotor wobbleGoalExtendMotor = null;
    DcMotor wobbleGoalRaiseMotor = null;
    Servo wobbleGoalGrippyThing = null;
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        wobbleGoalExtendMotor = hardwareMap.dcMotor.get("wobbleExtendo");
        wobbleGoalRaiseMotor = hardwareMap.dcMotor.get("wobbleLift");
        wobbleGoalGrippyThing = hardwareMap.servo.get("wobbleGrip");
        robot= new RobotClass(hardwareMap, telemetry, this);

        robot.wobbleGoalGrippyThingGrab();
        // here you detect rings

        int ringNmb = 0;
        waitForStart();

        robot.forward(0.5, -4.4);
        robot.pivotLeft(0.1, 17);
//        robot.shooterEngageAlt();
        robot.pause(1000);
//        shoot();
        robot.pivotRight(0.1, 6);
//        shoot();
        robot.pivotRight(.1, 6);
//        shoot();
//        robot.intakeServoEngage(0);
//        robot.stopShooting();
        robot.pivotRight(0.1, 5);

        if (ringNmb == 0) {
            robot.forward(0.5, -1.5);
            robot.strafeRight(0.5, 2);
            robot.depositWobbleGoal();
            robot.strafeLeft(0.5,2);
        } else if (ringNmb == 1) {
            robot.forward(0.5, -4);
            robot.depositWobbleGoal();
            robot.forward(0.5, 3);
        } else if (ringNmb == 4) {
            robot.forward(0.5,-5);
            robot.strafeRight(0.5, 2.2);
            robot.depositWobbleGoal();
            robot.strafeLeft(0.5,2.2);
            robot.forward(0.5,3.5);
        }

    }

    protected void shoot(){
        robot.shooterServo1(.8);
        robot.shooterServo2(.8);
        robot.pause(200);
        robot.intakeServoEngage(.9);
        robot.pause(800);
        robot.shooterServo2(0);
        robot.shooterServo1(0);
    }

}
