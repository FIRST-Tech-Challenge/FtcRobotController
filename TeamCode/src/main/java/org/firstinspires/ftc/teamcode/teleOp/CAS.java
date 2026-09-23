package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CAS")
public class CAS extends OpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private Servo frontLeftSteer;
    private Servo frontRightSteer;
    private Servo backLeftSteer;
    private Servo backRightSteer;

    // Distance between front and back wheels.
    private double wheelBase = 14.0;

    // Distance between left and right wheels.
    private double trackWidth = 14.0;

    //These are the servo positions where each wheel should point straight forward.
    private double FL_CENTER = 0.50;
    private double FR_CENTER = 0.50;
    private double BL_CENTER = 0.50;
    private double BR_CENTER = 0.50;


    /*
     * How much servo position represents
     * 90 degrees of wheel rotation.
     *
     * Example:
     *
     * 0.50 = center
     * 0.75 = +90 degrees
     * 0.25 = -90 degrees
     *
     * These are starting values.
     */

    private double SERVO_RANGE = 0.25;


    @Override
    public void init() {

        frontLeftDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "flDrive"
                );

        frontRightDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "frDrive"
                );

        backLeftDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "blDrive"
                );

        backRightDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "brDrive"
                );


        // =========================
        // GET STEERING SERVOS
        // =========================

        frontLeftSteer =
                hardwareMap.get(
                        Servo.class,
                        "flSteer"
                );

        frontRightSteer =
                hardwareMap.get(
                        Servo.class,
                        "frSteer"
                );

        backLeftSteer =
                hardwareMap.get(
                        Servo.class,
                        "blSteer"
                );

        backRightSteer =
                hardwareMap.get(
                        Servo.class,
                        "brSteer"
                );

        frontRightDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        backRightDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        frontLeftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontLeftSteer.setPosition(
                FL_CENTER
        );

        frontRightSteer.setPosition(
                FR_CENTER
        );

        backLeftSteer.setPosition(
                BL_CENTER
        );

        backRightSteer.setPosition(
                BR_CENTER
        );


        telemetry.addLine(
                "CAS Servo Swerve Ready"
        );

        telemetry.update();
    }


    @Override
    public void loop() {

        double forward =
                -gamepad1.left_stick_y;

        double strafe =
                gamepad1.left_stick_x;

        double rotation =
                gamepad1.right_stick_x;

        if (Math.abs(forward) < 0.05) {
            forward = 0;
        }

        if (Math.abs(strafe) < 0.05) {
            strafe = 0;
        }

        if (Math.abs(rotation) < 0.05) {
            rotation = 0;
        }

        double r =
                Math.hypot(
                        wheelBase,
                        trackWidth
                );


        double A =
                strafe -
                        rotation * (wheelBase / r);

        double B =
                strafe +
                        rotation * (wheelBase / r);

        double C =
                forward -
                        rotation * (trackWidth / r);

        double D =
                forward +
                        rotation * (trackWidth / r);

        double frontLeftSpeed =
                Math.hypot(B, D);

        double frontRightSpeed =
                Math.hypot(B, C);

        double backLeftSpeed =
                Math.hypot(A, D);

        double backRightSpeed =
                Math.hypot(A, C);

        double frontLeftAngle =
                Math.atan2(B, D);

        double frontRightAngle =
                Math.atan2(B, C);

        double backLeftAngle =
                Math.atan2(A, D);

        double backRightAngle =
                Math.atan2(A, C);

        double maxSpeed =
                Math.max(
                        Math.max(
                                frontLeftSpeed,
                                frontRightSpeed
                        ),
                        Math.max(
                                backLeftSpeed,
                                backRightSpeed
                        )
                );


        if (maxSpeed > 1.0) {

            frontLeftSpeed /=
                    maxSpeed;

            frontRightSpeed /=
                    maxSpeed;

            backLeftSpeed /=
                    maxSpeed;

            backRightSpeed /=
                    maxSpeed;
        }
        
        setModule(
                frontLeftDrive,
                frontLeftSteer,
                frontLeftSpeed,
                frontLeftAngle,
                FL_CENTER
        );

        setModule(
                frontRightDrive,
                frontRightSteer,
                frontRightSpeed,
                frontRightAngle,
                FR_CENTER
        );

        setModule(
                backLeftDrive,
                backLeftSteer,
                backLeftSpeed,
                backLeftAngle,
                BL_CENTER
        );

        setModule(
                backRightDrive,
                backRightSteer,
                backRightSpeed,
                backRightAngle,
                BR_CENTER
        );

        telemetry.addData(
                "Forward",
                "%.2f",
                forward
        );

        telemetry.addData(
                "Strafe",
                "%.2f",
                strafe
        );

        telemetry.addData(
                "Rotation",
                "%.2f",
                rotation
        );

        telemetry.addData(
                "FL Speed",
                "%.2f",
                frontLeftSpeed
        );

        telemetry.addData(
                "FR Speed",
                "%.2f",
                frontRightSpeed
        );

        telemetry.addData(
                "BL Speed",
                "%.2f",
                backLeftSpeed
        );

        telemetry.addData(
                "BR Speed",
                "%.2f",
                backRightSpeed
        );

        telemetry.update();
    }

    private void setModule(
            DcMotor driveMotor,
            Servo steeringServo,
            double speed,
            double angleRadians,
            double centerPosition
    ) {

        // Convert radians to degrees.
        double angleDegrees =
                Math.toDegrees(
                        angleRadians
                );

        if (angleDegrees > 90) {

            angleDegrees -= 180;

            speed = -speed;
        }

        if (angleDegrees < -90) {

            angleDegrees += 180;

            speed = -speed;
        }

        double servoPosition =
                centerPosition +
                        (angleDegrees / 90.0)
                                * SERVO_RANGE;

        servoPosition =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                servoPosition
                        )
                );

        steeringServo.setPosition(
                servoPosition
        );

        driveMotor.setPower(
                speed
        );
    }
}