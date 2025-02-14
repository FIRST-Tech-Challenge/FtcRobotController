package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "drivetrainAutoBasketProgram", group = "Drive")
public class drivetrainAutoBasketProgram extends LinearOpMode {
    DcMotorEx leftLiftMotor, rightLiftMotor;
    Servo basketServo;
    Servo intakePivotServo;

    static final int forwardMovement = 400;
    boolean noHighBasket = true;
    boolean isliftUp = false;
    int pauseSeperation = 0;
    boolean basket1stPos = false;
    boolean basket2ndPos = false;
    Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        // Initialize drivetrain
        // Initialize lift motors
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 1, 3, 30, 20, 0.8);
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            // stick to the back wall at a 20cm distance
            drivetrain.alignToWall(Drivetrain.WallType.BACK, 40);
            drivetrain.alignToWall(Drivetrain.WallType.LEFT, 40);
            drivetrain.update();

            telemetry.addData("isAlignedToWall: ", drivetrain.isMoving);
            telemetry.addData("horz: ", drivetrain.horizontalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("vert: ", drivetrain.verticalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("xWeights: ", drivetrain.xWeights.toString());
            telemetry.addData("yWeights: ", drivetrain.yWeights.toString());
            telemetry.addData("rWeights: ", drivetrain.rWeights.toString());
            telemetry.update();

            if (!drivetrain.isMoving) {
                break;
            }
        }
        drivetrain.stop();

        while (opModeIsActive()) {
            drivetrain.setAngle(-45);
            drivetrain.update();

            if (!drivetrain.isMoving) {
                break;
            }
        }

        drivetrain.stop(); // stop drivetrain motors when done
        while (noHighBasket) {
            highBasketProcess(); // do basket
        }
        // pick up block
    }

    public void setLiftPosition(int targetPosition) {
        leftLiftMotor.setTargetPosition(-targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setVelocity(2000);
        rightLiftMotor.setVelocity(2000);

        if (rightLiftMotor.getCurrentPosition() > 2890) {
            isliftUp = true;
        }
    }

    private void highBasketProcess() {
        if (!isliftUp) {
            setLiftPosition(3266); // low basket encoder positions
            telemetry.addData("Lift up", rightLiftMotor.getCurrentPosition());
            telemetry.update();
        }
        else if ((isliftUp) && (!basket2ndPos)) {
            basket();
            telemetry.addData("Basket", rightLiftMotor.getCurrentPosition());
            telemetry.update();
        }
        else if (basket2ndPos) {
            pauseSeperation += 1;
            if (pauseSeperation == 1) {
                // drivetrain.nudgeInDirection(0.0, 0.4);
                // drivetrain.stop();
            }
            setLiftPosition(0); // low basket encoder positions
        }
        else {
            noHighBasket = false;
        }
    }

    public void basket() {
        if (basket1stPos == false) {
            basketServo.setPosition(1.0);
        }
        else if ((basketServo.getPosition() > 0.45) && (basket1stPos == false)) {
            basket1stPos = true;
        }
        sleep(1500);
        basketServo.setPosition(0.0);
        if (basketServo.getPosition() < 0.075) {
            basket2ndPos = true;
            basket1stPos = true;
        }
        sleep(1500);
    }
}