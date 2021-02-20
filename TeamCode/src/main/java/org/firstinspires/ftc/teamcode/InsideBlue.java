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
        // here you detect rings, no rings=0 and so on

        int ringNmb = 1;
        waitForStart();

        robot.forward(0.5, -4);
        //shoot here
        robot.shooterServo1(1);
        robot.shooterServo2(1);
        robot.shooterEngage(5);
        robot.forward(0.5,-2);
        if (ringNmb == 0) {
            //strafe right
            robot.strafeRight(0.5, 2);
        } else if (ringNmb == 1) {
            robot.forward(0.5,-3);
            robot.moveWobbleGoalArm(0.7,0.4);
            robot.wobbleGoalGrippyThingRelease();
            robot.forward(0.5,2);
        } else if (ringNmb == 3) {
            robot.forward(0.5,-4);
            //forward+ strafe right+ wobble goal
        }

    }
}
