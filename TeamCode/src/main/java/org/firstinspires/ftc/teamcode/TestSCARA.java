package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Point;

@Autonomous(name="Test SCARA", group="Linear Opmode")
public class TestSCARA extends LinearOpMode {

    Servo scara1, scara2, servoRotate, gripper;
    SCARAController controller;
    SCARAController.ClawPosition currentClawPosition;
//    SCARAController.ArmAngles armAngles;
//    SCARAController.ServoPositions servoPositions;

    final static double SERVO_SPEED = 0.15; // servo counts per second
    final static double ROTATE_SPEED = 180.0/270; // degrees per second for rotate servo
    double rotateServo = .397; // 0.237;

    //    double x, y;
    final static double SERVO1_THETA1_IS_0 = 0;

    int step = 0;
    boolean toggle = false;
    int ind = 0;
    boolean ind_toggle = false;

    public void runOpMode () {
        scara1 = hardwareMap.servo.get("servoLink1");
        scara2 = hardwareMap.servo.get("servoLink2");
        servoRotate = hardwareMap.servo.get("gripperRotation");
        gripper = hardwareMap.servo.get("grabberServo");

        controller = new SCARAController(120, 120, telemetry);
        currentClawPosition = controller.new ClawPosition(controller.clawInsideRobot);

//        currentClawPosition = controller.new ClawPosition(controller.clawInsideRobot);

        while(!isStarted()) {
            telemetry.addData("SCARA ready ", "");
            telemetry.update();

            //throttle to 10Hz loop to avoid burning excess CPU cycles for no reason
            sleep(100);
        }

//        double SERVO1_START = controller.clawInsideRobot.arm1Servo;
//        double SERVO2_START = controller.clawInsideRobot.arm2Servo;

        double lastTime = getRuntime();
        //keep opmode running so print block location can be read from screen
        while (opModeIsActive()) {
            double currentTime = getRuntime();

            if (gamepad1.left_bumper) {
                currentClawPosition.moveBy(gamepad1.left_stick_x, -gamepad1.left_stick_y, currentTime - lastTime);
            }

            rotateServo += gamepad1.right_stick_x * ROTATE_SPEED * (currentTime - lastTime);
            if (rotateServo > 1) rotateServo = 1;
            if (rotateServo < 0) rotateServo = 0;

            // manipulate servos directly
            if (gamepad1.x) {
                currentClawPosition.servoPositions.servo1 += SERVO_SPEED * (currentTime - lastTime);
                if (currentClawPosition.servoPositions.servo1 > 1)
                    currentClawPosition.servoPositions.servo1 = 1;
            }
            if (gamepad1.y) {
                currentClawPosition.servoPositions.servo1 -= SERVO_SPEED * (currentTime - lastTime);
                if (currentClawPosition.servoPositions.servo1 < 0)
                    currentClawPosition.servoPositions.servo1 = 0;
            }


            if (gamepad1.a) {
                currentClawPosition.servoPositions.servo2 += SERVO_SPEED * (currentTime - lastTime);
                if (currentClawPosition.servoPositions.servo2 > 1)
                    currentClawPosition.servoPositions.servo2 = 1;
            }
            if (gamepad1.b) {
                currentClawPosition.servoPositions.servo2 -= SERVO_SPEED * (currentTime - lastTime);
                if (currentClawPosition.servoPositions.servo2 < 0)
                    currentClawPosition.servoPositions.servo2 = 0;
            }

            if (gamepad1.dpad_up) {
                gripper.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                gripper.setPosition(0);
            }

            if (gamepad1.right_bumper) {
                currentClawPosition.moveSequence(controller.INSIDE_ROBOT_TO_DELIVERY, currentTime - lastTime);
//                currentClawPosition.moveTo(SCARAController.MIDLINE, SCARAController.DELIVER_Y_DISTANCE, currentTime - lastTime);
            }
            if (gamepad1.right_trigger > 0.5) {
                currentClawPosition.moveSequence(controller.DELIVERY_TO_INSIDE_ROBOT, currentTime - lastTime);
//                currentClawPosition.moveTo(SCARAController.MIDLINE, SCARAController.PICK_UP_Y_DISTANCE, currentTime - lastTime);
            }


            if (gamepad1.left_trigger > 0.5) {
                if (!toggle) {
                    step++;
                    if (step >= 4) step = 0;
                    toggle = true;
                }
                switch (step) {
                    case 0:
                        currentClawPosition.moveTo(SCARAController.MIDLINE, SCARAController.PICK_UP_Y_DISTANCE, currentTime - lastTime);
                        break;
                    case 1:
                        currentClawPosition.moveTo(SCARAController.MIDLINE, -85, currentTime - lastTime);
                        break;
                    case 2:
                        currentClawPosition.moveTo(SCARAController.MIDLINE, 0, currentTime - lastTime);
                        break;
                    case 3:
                        currentClawPosition.moveTo(SCARAController.MIDLINE, SCARAController.DELIVER_Y_DISTANCE, currentTime - lastTime);
                        break;
                }
            } else {
                toggle = false;
            }

            SCARAController.Sequence testSequence = controller.INSIDE_ROBOT_TO_DELIVERY;
            if (gamepad1.left_stick_y < -.5) {
                if (!ind_toggle) {
                    ind++;
                    if (ind >= testSequence.coordinatesList.size()) {
                        ind = testSequence.coordinatesList.size() - 1;
                    }
                    ind_toggle = true;
                }
                currentClawPosition.armAngles.copy(testSequence.angleList.get(ind));
                currentClawPosition.servoPositions.updateFromControlAngles(currentClawPosition.armAngles);
            } else if (gamepad1.left_stick_y > .5) {
                if (ind_toggle) {
                    ind--;
                    if (ind < 0) ind = 0;
                    ind_toggle = true;
                }
                currentClawPosition.armAngles.copy(testSequence.angleList.get(ind));
                currentClawPosition.servoPositions.updateFromControlAngles(currentClawPosition.armAngles);
            } else {
                ind_toggle = false;
            }
            for (int i = 0; i <=10; i++) {

                SCARAController.Coordinates c =controller.INSIDE_ROBOT_TO_DELIVERY.coordinatesList.get(i);
                SCARAController.ArmAngles a = controller.INSIDE_ROBOT_TO_DELIVERY.angleList.get(i);
                telemetry.addData("P"+i,c.x+", "+c.y+"   "+a.angle1 *180/Math.PI+", "+a.angle2*180/Math.PI);
            }


            // gamepad 2
            if (gamepad2.x) { // inside robot
                currentClawPosition.armAngles.angle1 = -85.0 * Math.PI / 180;
                currentClawPosition.armAngles.angle2 = -70 * Math.PI / 180;
                currentClawPosition.servoPositions.updateFromControlAngles(currentClawPosition.armAngles);
            }
            if (gamepad2.a) { // under bridge
                currentClawPosition.armAngles.angle1 = -90.0 * Math.PI / 180;
                currentClawPosition.armAngles.angle2 = -160.0 * Math.PI / 180;
                currentClawPosition.servoPositions.updateFromControlAngles(currentClawPosition.armAngles);
            }
            if (gamepad2.b) { // outside robot
                currentClawPosition.armAngles.angle1 = 160.0 * Math.PI / 180;
                currentClawPosition.armAngles.angle2 = -80.0 * Math.PI / 180;
                currentClawPosition.servoPositions.updateFromControlAngles(currentClawPosition.armAngles);
            }

            scara1.setPosition(currentClawPosition.servoPositions.servo1);
            scara2.setPosition(currentClawPosition.servoPositions.servo2);
            servoRotate.setPosition(rotateServo);

            telemetry.addData("x", currentClawPosition.coordinates.x);
            telemetry.addData("y", currentClawPosition.coordinates.y);
            telemetry.addData("theta1", currentClawPosition.armAngles.angle1 + ", " + currentClawPosition.armAngles.angle1 * 180/Math.PI );
            telemetry.addData("theta2", currentClawPosition.armAngles.angle2 + ", " + currentClawPosition.armAngles.angle2 * 180/ Math.PI);
            telemetry.addData("rotate", rotateServo);

            telemetry.addData("servo1 ticks per radian", controller.servo1TicksPerRadian + ", " + controller.servo1TicksPerRadian * Math.PI / 180);
            telemetry.addData("servo2 ticks per radian", controller.servo2TicksPerRadian + ", " + controller.servo2TicksPerRadian * Math.PI / 180);
            telemetry.addData("servo1", currentClawPosition.servoPositions.servo1);
            telemetry.addData("servo2", currentClawPosition.servoPositions.servo2);
            telemetry.addData("loop time", currentTime - lastTime);
            telemetry.addData("current time", currentTime);


            telemetry.addData("claw inside", controller.clawInsideRobot.armAngles.angle1 * 180 / Math.PI + ", " + controller.clawInsideRobot.armAngles.angle2* 180 / Math.PI );
            telemetry.addData("claw under bridge", controller.clawUnderBridge.armAngles.angle1* 180 / Math.PI  + ", " + controller.clawUnderBridge.armAngles.angle2* 180 / Math.PI );
            telemetry.addData("claw outside", controller.clawOutsideRobot.armAngles.angle1* 180 / Math.PI  + ", " + controller.clawOutsideRobot.armAngles.angle2* 180 / Math.PI );
            telemetry.update();
//            sleep(50);
            lastTime = currentTime;
        }
    }
}