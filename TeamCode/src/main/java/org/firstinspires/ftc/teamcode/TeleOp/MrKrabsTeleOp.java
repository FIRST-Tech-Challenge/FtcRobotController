package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

    private DcMotor vertMotor;

    int deliveryPosition;
    int deliveryError;

    private Servo claw;

    private boolean controlsRelinquished = false;
    private final double DRIVE_DEADZONE = 0.05;
    private final double SCORE_SPEED_SCALAR = 0.2;

    private double speedScalar = 1;


    @Override
    public void runOpMode() {
        super.runOpMode();

        drivetrainFunctions = new DrivetrainFunctions(this);

        vertMotor = hardwareMap.get(DcMotor.class, "deliverySlide");
        vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vertMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");

//        delivery = new Delivery(this, true);


        waitForStart();

        while (opModeIsActive()){

            deliveryPosition = vertMotor.getCurrentPosition();
            deliveryError = targetPosition - deliveryPosition;
            targetPosition = Math.max(Math.min(targetPosition, 2300), 0);

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
                float leftX = -gamepad1.left_stick_x;
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

            switch (deliveryState) {
                case DELIVERY_START:
                    if (gamepad2.a) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 1470;//Intake from observation wall; FIXME: VALUE IS WRONG
                    }
                    if (gamepad2.x) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 2000;//hook onto high bar
                    }
                    if (gamepad2.y) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_LIFT;
                        targetPosition = 300;//SET_3_HEIGHT;
                    }
                    if (gamepad2.b) {
                        deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_RETRACT;
                        deliveryTimer.reset();
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
                    if(Math.abs(deliveryError) <= 5) deliveryState = DeliveryState.DELIVERY_START;
                    break;

                case DELIVERY_RETRACT:
                    targetPosition = 0;
                    if(deliveryPosition <= 10) {
                        vertMotor.setPower(0);
                        deliveryState = DeliveryState.DELIVERY_START;
                    }

                    break;

            }

            telemetry.addData("Delivery State: ", deliveryState);
            telemetry.addData("Delivery Position: ", deliveryPosition);
            telemetry.addData("Delivery Error: ", deliveryError);
            telemetry.addData("Delivery Timer: ", deliveryTimer.seconds());
            telemetry.addData("Delivery Power: ", vertMotor.getPower());



            //MANUAL
            if (Math.abs(gamepad2.right_stick_y) >= DRIVE_DEADZONE) {
                targetPosition = Math.max(Math.min(targetPosition, deliveryPosition+200), deliveryPosition-200);
                deliveryState = MrKrabsTeleOp.DeliveryState.DELIVERY_START;

                if (deliveryError < 1000) {
                    targetPosition -= gamepad2.right_stick_y * 35;
                }
            }


            if(gamepad2.right_bumper){
                clawClose();
            }else{
                clawOpen();
            }

            squidToPointDelivery();
            telemetry.addData("Target Position: ", targetPosition);

            telemetry.addLine();
            telemetry.addData("Claw Position: ", claw.getPosition());
            telemetry.update();
        }
    }



    void squidToPointDelivery()
    {
        double h = 1.5;

        final double MAX_ERROR_FOR_FULL_POWER = 500.0;  //adjust
        double norm = Math.min(1.0, Math.abs(deliveryError) / MAX_ERROR_FOR_FULL_POWER);
        double mag = Math.sqrt(h*norm);
        double power = Math.copySign(mag, deliveryError);
        final double MIN_POWER = 0.08;  // tweak or set to 0 to disable
        if (mag > 0 && mag < MIN_POWER) {
            power = Math.copySign(MIN_POWER, deliveryError);
        }
        vertMotor.setPower(power);
    }

    void clawClose()
    {
        claw.setPosition(0.235);
    }

    void clawOpen()
    {
        claw.setPosition(0.0);
    }
}
