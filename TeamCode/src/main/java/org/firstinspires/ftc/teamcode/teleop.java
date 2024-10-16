package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp", group = "Teleops")
    public class teleop extends OpMode {

    // Field centric stuff
    boolean usingFC = false;
    double initialAngle = 0;
    double prevFCPressTime = 0;


    double prevPos = 0;

    // Mechanism stuff
    double CLOSED_OT_POS = 0;
    double OPEN_OT_POS = 1;
    double UP_OT_FLIP_POS = 0;
    double DOWN_OT_FLIP_POS = 1;
    double CLOSED_IT_POS = 0;
    double OPEN_IT_POS = 1;
    double UP_IT_FLIP_POS = 0;
    double DOWN_IT_FLIP_POS = 1;

    // Total time. Never reset.
    ElapsedTime totalTime = new ElapsedTime();

    public IMU imu;


    // Servos and motors for outtake/intake.
    DcMotor outTakeLift;
    Servo outTakeClaw;
    Servo outTakeFlip;

    DcMotor inTakeLift;
    Servo inTakeClaw;
    Servo inTakeFlip;
    //BNO055IMU.Parameters pars = new BNO055IMU.Parameters();
    //Orientation angles;

    @Override
    public void init(){
        imu = hardwareMap.get(IMU.class, "imu");
        // change it to match the actual orientation of the rev control hub
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        outTakeLift = hardwareMap.dcMotor.get("otl");
        outTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outTakeClaw = hardwareMap.servo.get(("otc"));
        outTakeFlip = hardwareMap.servo.get(("otf"));

        inTakeLift = hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inTakeClaw = hardwareMap.servo.get(("itc"));
        inTakeFlip = hardwareMap.servo.get(("itf"));

    }
    public double returnGyroYaw()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getAngle()
    {
        return Math.toDegrees(returnGyroYaw());
    }

    public void setOutTakeLift(){
        outTakeLift.setPower(gamepad2.left_stick_y);
    }

    public void setOutTakeClawGrab(){
        if (gamepad2.a)
            outTakeClaw.setPosition(CLOSED_OT_POS);
        if (gamepad2.b)
            outTakeClaw.setPosition(OPEN_OT_POS);
    }

    public void setOutTakeFlip(){
        if (gamepad2.x)
            outTakeClaw.setPosition(DOWN_OT_FLIP_POS);
        if (gamepad2.y)
            outTakeClaw.setPosition(UP_OT_FLIP_POS);
    }

    public void setInTakeClawGrab(){
        if (gamepad2.dpad_left)
            inTakeClaw.setPosition(CLOSED_IT_POS);
        if (gamepad2.dpad_right)
            inTakeClaw.setPosition(OPEN_IT_POS);
    }

    public void setInTakeFlip(){
        if (gamepad2.dpad_up)
            inTakeClaw.setPosition(UP_IT_FLIP_POS);
        if (gamepad2.dpad_down)
            inTakeClaw.setPosition(DOWN_IT_FLIP_POS);
    }

    public void setInTakeLift(){
        inTakeLift.setPower(gamepad2.right_stick_y);
    }

    public double getTrueAngle(double heading)
    {
        double angle = returnGyroYaw();
        if (Math.abs(heading - angle) < 180 )
        {
            return heading - angle;
        }
        if (heading - angle >= 180)
        {
            return  360 - heading - angle;
        }
        if (angle - heading <= -180)
        {
            return angle - heading + 360;
        }
        return heading - angle;
    }
    @Override
    public void loop() {
        File file;
        Scanner scan;
        String fileName = "/sdcard/FIRST/PathTest.txt";
        String position;
        int positions = 0;
        String[][] path;
        double dYdX;
        double totalDistance;
        double distance;
        double theta;
        double xCoordinate;
        double yCoordinate;
        // robot centric

        // declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backR");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setOutTakeLift();
        setInTakeLift();
        setInTakeClawGrab();
        setInTakeFlip();
        setOutTakeFlip();
        setOutTakeClawGrab();
        
        if (gamepad1.y) {

            file = new File(fileName);
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            while(scan.hasNextLine()){
                positions++;
                scan.nextLine() ;
            }
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            path = new String[positions][7];
            for (int i = 0; i < path.length; i++) {
                if (scan.hasNextLine()){
                    position = scan.nextLine();
                    position = position.replaceAll("[^0-9. -]", "");
                    path[i] = position.split(" ");
                }
                try{
                    dYdX = Double.parseDouble(path[i][1]);
                    totalDistance = Double.parseDouble(path[i][2]);
                    distance = Double.parseDouble(path[i][3]);
                    theta = Double.parseDouble(path[i][4]);
                    xCoordinate = Double.parseDouble(path[i][5]);
                    yCoordinate = Double.parseDouble(path[i][6]);
                } catch (NumberFormatException e){
                    continue;
                }
                telemetry.addLine("dYdX = " + dYdX);
                telemetry.addLine("totalDist = " + totalDistance);
                telemetry.addLine("dist = " + distance);
                telemetry.addLine("theta = " + theta);
                telemetry.addLine("x = " + xCoordinate);
                telemetry.addLine("y = " + yCoordinate);
                telemetry.addLine(" ");
            }
            telemetry.update();
        }


        // field centric activation
        if (gamepad1.b && totalTime.milliseconds() - 500 > prevFCPressTime) {
            if (!usingFC) {
                initialAngle = getAngle();
                usingFC = true;
            } else {
                usingFC = false;
            }
            prevFCPressTime = totalTime.milliseconds();
        }

        if (usingFC) {
            // this would be start on Xbox (changeable)
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;



            double trueDiff = getTrueAngle(initialAngle);
            double joyAngle = Math.toDegrees(Math.atan2(y, x));
            double trueJoy = 90 - joyAngle;
            if (trueJoy > 180)
            {
                trueJoy = (trueJoy - 360);
            }
            double cos = Math.cos(Math.toRadians(trueJoy - trueDiff));
            double sin = Math.sin(Math.toRadians(trueJoy - trueDiff));

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            double power = Math.min(Math.abs(y) + Math.abs(x), 1);
            frontLeftMotor.setPower((power * cos + power * sin + rx));
            backLeftMotor.setPower((power * cos - power * sin + rx));
            frontRightMotor.setPower((power * cos - power * sin - rx));
            backRightMotor.setPower((power * cos + power * sin - rx));
        }
        else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(-backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        // field centric activation
        if (gamepad1.b) {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // change it to match the actual orientation of the rev control hub
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // this would be start on Xbox (changeable)
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // rotate the movement direction to counter the robot's true orientation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

    }

}

}
