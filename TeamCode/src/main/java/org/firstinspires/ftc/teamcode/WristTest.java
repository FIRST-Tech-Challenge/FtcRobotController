package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WristTest extends LinearOpMode {

    //new 10.1.1

    //Servo wristLeft;
    //Servo wristRight;
    //Servo armLeft, armRight;
    Wrist wrist;
    Arm arm;
    Gripper gripper;

    @Override
    public void runOpMode() throws InterruptedException {

        /*wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");


        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");

        wristLeft.setDirection(Servo.Direction.FORWARD);
        wristRight.setDirection(Servo.Direction.REVERSE);//club test2
        //club test fr5

        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.FORWARD);


        wrist = new Wrist(wristLeft, wristRight);
        arm = new Arm(armLeft, armRight);*/
        arm = new Arm(hardwareMap);
        arm.goToAngle(0);

        wrist = new Wrist(hardwareMap);
        wrist.setWristToTarget(0,0, 0);

        gripper = new Gripper(hardwareMap);
        //gripper.open();

        double wristPitchTarget;
        double wristRollTarget;
        waitForStart();
        while (opModeIsActive()) {


            /*if (gamepad1.a) {
                wristPitchTarget = 45;
                wristRollTarget = 45;
            } else if (gamepad1.b) {
                wristPitchTarget = -60;
                wristRollTarget = 90;
            } else if (gamepad1.x) {
                wristPitchTarget = 90;
                wristRollTarget = 90;
            } else if (gamepad1.y) {
                wristPitchTarget = -90;
                wristRollTarget = 90;
            } else if (gamepad1.right_bumper) {
                wristPitchTarget = -90;
                wristRollTarget = 0;
            } else {
                wristPitchTarget = 0;
                wristRollTarget = 0;
            }*/

            /*if (gamepad1.left_bumper) {
                armLeft.setPosition(0.8);
                armRight.setPosition(0.8);
            } else {
                armLeft.setPosition(0.3);
                armRight.setPosition(0.3);

            }*/
            double armAngleTarget = -gamepad2.right_stick_y * 180;
            arm.goToAngle(armAngleTarget);
            wristPitchTarget = -gamepad1.left_stick_y * 90;
            wristRollTarget = gamepad1.right_stick_x * 112;

            wrist.setWristToTarget(wristPitchTarget, wristRollTarget, arm.armAngleCurrent());

            if (gamepad1.right_bumper) {
                gripper.open();
            } else if (gamepad1.left_bumper) {
                gripper.close();
            }

            telemetry.addData("wristLeft", wrist.left.getPosition());
            telemetry.addData("wristRight", wrist.right.getPosition());
            telemetry.addData("wristRobotRel", wristPitchTarget);
            telemetry.addLine(wrist.toString());
            //telemetry.addData("armRight", armRight.getPosition());
            telemetry.addData("armLeft", arm.left.getPosition());

            telemetry.addData("ARM ANGLE", arm.armAngleCurrent());
            //telemetry.addData("wristLeft", wristLeft.getPosition());
            //telemetry.addData("wristRight", wristRight.getPosition());
            //telemetry.addData("leftDegrees", wrist.servoToDeg(wristLeft.getPosition()));
            //telemetry.addData("rightDegrees", wrist.servoToDeg(wristRight.getPosition()));

            //telemetry.addLine(String.format("\nPITCH %.1f°", wrist.wristPitchCurrent()));
            //telemetry.addLine(String.format("ROLL %.1f°", wrist.wristRollCurrent()));
            telemetry.addLine(gripper.toString());
            telemetry.addData("gripperPos", gripper.front.getPosition());


            //telemetry.addData("\nPITCH", wristPitchCurrent());
            //telemetry.addData("ROLL", wristRollCurrent());

            telemetry.update();
        }

    }
}
