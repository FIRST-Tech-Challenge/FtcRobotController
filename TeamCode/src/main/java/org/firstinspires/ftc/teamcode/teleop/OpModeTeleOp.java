package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Teleop", group = "Furious Frog")
public class OpModeTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure your ID's match your configuration

        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> allDeviceMappings = hardwareMap.allDeviceMappings;

        allDeviceMappings.forEach(d -> {
            System.out.println(d.getDeviceTypeClass().getCanonicalName());
        });
        int armInit, armInit2;
        double clawLowR= 0.05;
        double clawHighR = 0.6;
        double clawLowL= 0.3;
        double clawHighL = 0.70;
        double armOffset = 0.20;
        double ARM_SPEED = 0.01;

        double CLAW_SPEED = 0.5;                 // sets rate to move servo
        int ClawStatusR = 0;
        int ClawStatusL = 1;

        TouchSensor touchSensor, touchClaw;  // Touch sensor Object

        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        MacanumWheelsTeleop wheels = new MacanumWheelsTeleop(hardwareMap, telemetry);
        Servo clawServoR = hardwareMap.servo.get("clawServo");
        Servo clawServoL = hardwareMap.servo.get("clawServo2");
        Servo droneServo = hardwareMap.servo.get("droneServo");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("armMotor2");

        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        touchClaw = hardwareMap.get(TouchSensor.class, "sensor_claw");
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServoR.setPosition(clawLowR);
        clawServoL.setPosition(clawHighL);

        droneServo.setPosition(0);

       // armInit = armMotor.getTargetPosition();
       // armInit2 = armMotor2.getTargetPosition();
        //droneServo.setPosition(-1);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double chassisY = getChassisY();
            double chassisX = getChassisX();
            double chassisTurn = gamepad1.right_stick_x;
            //  System.out.println("chassisTurn is " + chassisTurn);
            //  double armTurn = gamepad2.right_stick_x;
            wheels.move(chassisX, chassisY, chassisTurn);

            //System.out.println("dpadUp is " + dpadUp);
            //System.out.println("dpadDown is " + dpadDown);
           // telemetry.addData("claw", "Offset = %.2f", clawServo.getPosition());

            // Use gamepad dpad up and down open and close the claw
            if (gamepad2.right_bumper) {
                      clawServoR.setPosition(clawLowR);
                } else if(gamepad2.right_trigger > 0.0) {
                      clawServoR.setPosition(clawHighR);
                }

                //clawRight = Range.clip(clawRight, 0.0, 1.0);

            //}
                //else if (gamepad2.right_bumper && touchSensor.isPressed())
            if (gamepad2.left_bumper){
                  clawServoL.setPosition(clawHighL);
                } else if(gamepad2.left_trigger > 0.0){
                    clawServoL.setPosition(clawLowL);
                }


            if (gamepad2.dpad_up)
                armOffset += ARM_SPEED;
            else if (gamepad2.dpad_down)
                armOffset -= ARM_SPEED;
            armOffset = Range.clip(armOffset, 0.0, 1.0);

            if (gamepad2.y) {
                droneServo.setPosition(1);
                telemetry.addData("Drone Servo", "Pressed gamepad2");
                telemetry.update();

            }

            if (gamepad1.y) {
                droneServo.setPosition(0);
                telemetry.addData("Drone Servo", "Pressed gamepad1");
                telemetry.update();
            }


            double armLY = gamepad2.left_stick_y;
            double powerArmLY = armLY * -1.0;
             if (touchSensor.isPressed()){
                 if(armLY > 0)
                      powerArmLY = 0;
              }
            armMotor.setPower(powerArmLY);
            //System.out.println("armMotor is " + armMotor);

            if (armMotor2 != null) {
                double armRY = gamepad2.right_stick_y;
                double armRightY = armRY;
                double powerArmLY2 = armLY * -armOffset;
                //if (powerArmLY2 > 0) {
                 //   powerArmLY2 = powerArmLY2 * 1;
               // }
                if (touchClaw.isPressed()) {
                    if (armRY > 0)
                        armRightY = 0;
                }
                armMotor2.setPower(powerArmLY2);
                armMotor2.setPower(armRightY);

            }
            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

           // armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           // armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Bring back to starting position to grab the pixel


            if (gamepad2.x) {
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
           // telemetry.addData("claw", "R Offset = %.2f, R Offset = %.2f", clawRight, clawLeft);
            telemetry.addData("clawfrom servo", "OffsetR = %.2f, Offsetl = %.2f", clawServoR.getPosition(), clawServoL.getPosition());
            telemetry.addData("Motors", "armoffset= %.2f and powerArmY =%.2f", armOffset,armLY);
            telemetry.addData("Motors",  "armMotor = %7d and armMotor2= %7d ",armMotor.getCurrentPosition(), armMotor2.getCurrentPosition());
            telemetry.update();

        }
    }

    private double getChassisX() {
        double chassisX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        //  System.out.println("gamepad1.left_stick_x is " + chassisX);
        return chassisX;
    }

    private double getChassisY() {
        double chassisY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        //  System.out.println("gamepad1.left_stick_y is " + chassisY);
        return chassisY;
    }
}
