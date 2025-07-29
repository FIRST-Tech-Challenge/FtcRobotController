package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Delivery;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RoboMom;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="MrKrabs Teleop", group="AAA")
public class MrKrabsTeleOp extends RoboMom {

    private ElapsedTime deliveryTimer = new ElapsedTime();

    DrivetrainFunctions drivetrainFunctions = null;
//    Intake intake = null;
    Delivery delivery = null;
    public enum DeliveryState{
        DELIVERY_START,
        DELIVERY_LIFT,
        DELIVERY_RETRACT
    }
    private DeliveryState deliveryState = DeliveryState.DELIVERY_START;
    private int slidePosition = 0;

    private int targetPosition = 0;

    private int targetPitch = 0;
    private double extension = 0.25;
    private int PITCH_INCREMENT = 10;

    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;

    //This is the code to add rr actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void runOpMode() {
        super.runOpMode();

        delivery = new Delivery(this, false);

        drivetrainFunctions = new DrivetrainFunctions(this);

//        intake = new Intake(this);


//        intake.unbrakePitch();

        waitForStart();

        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();

            //driver 1
            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            if (!controlsRelinquished) {
                float leftX = gamepad1.left_stick_x;
                float leftY = gamepad1.left_stick_y;
                float rightX = gamepad1.right_stick_x;
                float rightY = gamepad1.right_stick_y;
                if (Math.abs(leftX) > DRIVE_DEADZONE || Math.abs(leftY) > DRIVE_DEADZONE || Math.abs(rightX) > DRIVE_DEADZONE || Math.abs(rightY) > DRIVE_DEADZONE*2) {
                    if(Math.abs(rightY) > DRIVE_DEADZONE*2) {
                        drivetrainFunctions.Move(leftX, rightY, rightX, speedScalar);
                    }else{
                        drivetrainFunctions.Move(leftX, leftY, rightX, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }
            }

            //driver 2
            slidePosition = delivery.getMotorPosition();
            targetPosition = delivery.getMotorTargetPosition();


            switch (deliveryState) {
                case DELIVERY_START:
                    if (gamepad2.a) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 1470;//Intake from observation wall; FIXME: VALUE IS WRONG
                    }
                    if (gamepad2.x) {
//                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
//                        targetPosition = 4120;//hook onto high bar
                    }
                    if (gamepad2.y) {
//                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
//                        targetPosition = 300;//SET_3_HEIGHT;
                    }
                    if (gamepad2.b) {
//                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_RETRACT;
//                        deliveryTimer.reset();
                    }

//                    if (gamepad2.right_bumper && !dumped && deliveryFunctions.getMotorPositionByIndex(0) > deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS) {
//                        deliveryState = DeliveryState.DELIVERY_DUMP;
//                        dumped = true;
//                        deliveryTimer.reset();
//                        deliveryFunctions.Dump(1);
//                        secondDumped = false;
//                    }
                    break;

                case DELIVERY_LIFT:
                    /*
                    //if both motors are within stop threshold
                    if
                    (targetPosition - leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                            targetPosition - rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {
                        deliveryState = CenterStageTeleOp.DeliveryState.DELIVERY_START;
                        dumped = false;
                        secondDumped = false;
                    } else {
                        //Still going
                    }
                    */
                    break;

                case DELIVERY_RETRACT:
                    /*
                    retracting = true;
                    if(deliveryTimer.seconds() <= 0.5){
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                        break;
                    }else{
                        targetPosition = LIFT_LOW;
                        if (deliveryFunctions.getMotorPositionByIndex(0) < deliveryFunctions.CARRIAGE_DODGE) {
                            deliveryFunctions.SetWristPosition(deliveryFunctions.servoIn);
                        } else {
                            deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                        }

                        //if both motors are within stop threshold
                        if
                        (leftMotorPosition - targetPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                                &&
                                rightMotorPosition - targetPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {

                            deliveryState = CenterStageTeleOp.DeliveryState.DELIVERY_START;
                            retracting = false;
                        }
                    }

                     */

                    break;

            }

            telemetry.addData("Delivery State: ", deliveryState);
            telemetry.addData("Delivery Timer: ", deliveryTimer.seconds());


            //Adds rr actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            //MANUAL
            if (Math.abs(gamepad2.right_stick_y) >= DRIVE_DEADZONE) {
                deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_START;

                if (slidePosition > delivery.lowPosition) {
                    targetPosition -= gamepad2.right_stick_y * 35;
                } else {
                    targetPosition -= gamepad2.right_stick_y * 25;
                }
            }

            if (Math.abs(gamepad2.left_stick_y) >= DRIVE_DEADZONE) {
                if(gamepad2.left_stick_y > 0){
//                    intake.incrementTargetAngleTicks(-PITCH_INCREMENT);
                } else if(gamepad2.left_stick_y < 0){
//                    intake.incrementTargetAngleTicks(+PITCH_INCREMENT);
                }
            }

            if(gamepad2.dpad_left){
//                intake.setTargetAngleTicks(0);
            }

            if(gamepad2.dpad_right){
//                intake.setTargetAngleTicks(750);
            }

            float rightTrigger = gamepad2.right_trigger;
            float leftTrigger = gamepad2.left_trigger;
            if(rightTrigger > 0){
                extension += rightTrigger/100;
                extension = Math.max(0.01, Math.min(0.25, extension));
//                intake.setTargetLengthServo(extension);
            } else if(leftTrigger > 0){
                extension -= leftTrigger/100;
                extension = Math.max(0.01, Math.min(0.25, extension));
//                intake.setTargetLengthServo(extension);
            }

//            intake.updateAngle();
//            intake.updateLength();

            if(gamepad2.right_bumper){
                delivery.clawClose();
            }else{
                delivery.clawOpen();
            }

            if(gamepad2.x){
//                intake.setEndEffectorSpeed(0);//in
            }else if(gamepad2.b){
//                intake.setEndEffectorSpeed(1);//out
            }else{
//                intake.setEndEffectorSpeed(0.5f);//stop
            }

            targetPosition = Math.max(delivery.bottomPosition, Math.min(delivery.topDelta, targetPosition));
            delivery.setSlidesTargetPosition(targetPosition);

            delivery.PControlPower(1);

            //telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Slide Motor Position (in): ", String.format( "%.2f", delivery.getMotorPositionInches()) );
            telemetry.addData("Slide Motor Target (in): ", String.format( "%.2f", delivery.getMotorTargetPositionInches()) );

            telemetry.addData("Target Position in DeliveryFunctions: ", delivery.getMotorTargetPosition());
            telemetry.addData("Slide Motor Position: ", slidePosition);

            telemetry.addLine();
            telemetry.addLine();
//            telemetry.addData("Intake Extension: ", intake.getExtensionServoPosition());
        }
    }
}
