package org.firstinspires.ftc.teamcode.TANVIII;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class AutoMode extends LinearOpMode {

    Robot robot;

    //wheel measurements
    final double PPR = 537.7;
    final double motorToWheelRatio = 1.4;
    final double wheelDiaMm = 96;
    final double PI = 3.14159;
    final double wheelCircIn = wheelDiaMm * PI / 25.4; //~11.87

    double tickToIn = motorToWheelRatio * wheelCircIn / PPR; // #ticks / tickToInches = #inches
    double inToTick = 33; //PPR / (motorToWheelRatio * wheelCircIn); //~32.357

    public double bigAbsVal(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }
        return max;
    }

    //TODO: should put it in robot class?
    private void autoDrive (double straightSpeed, double centerTurnSpeed, double mecanumSpeed) {
        //inches * tickToIn
        /*
        straightSpeed positive is forward
        centerTurnSpeed positive is clockwise turn
        mecanumSpeed positive is right (from back of bot)
         */
        //set power
        double flpr = -(straightSpeed) - (centerTurnSpeed) - (mecanumSpeed);
        double frpr = (straightSpeed) - (centerTurnSpeed) - (mecanumSpeed);
        double blpr = -(straightSpeed) - (centerTurnSpeed) + (mecanumSpeed);
        double brpr = (straightSpeed) - (centerTurnSpeed) + (mecanumSpeed);

        //scaling
        double bigPr = bigAbsVal(flpr, frpr, blpr, brpr);

        if (Math.abs(bigPr) > 1) {
            double scaleFactor = Math.abs(bigPr);
            flpr /= scaleFactor;
            frpr /= scaleFactor;
            blpr /= scaleFactor;
            brpr /= scaleFactor;
        }
        robot.setMotorPower(flpr, frpr, blpr, brpr);
    }
    /*
    public void encoderStraight(double inches) {
        double flTicksNow = Math.abs(robot.getEncoderPosition("fl"));
        double flTicksEnd = inches * inToTick + flTicksNow; //rip ticksToIn :(

        while (opModeIsActive()) {
            double KP = 0.007; //0.0027;
            flTicksNow = Math.abs(robot.getEncoderPosition("fl"));

            double flError = flTicksEnd - flTicksNow;
            if (Math.abs(flError) < 5) {
                break;
            }

            double flPower = KP * flError;
            if (flPower > 1) {
                flPower = 1;
            } else if (flPower < -1) {
                flPower = -1;
            }
            autoDrive(flPower/4, 0,0);
        }

        autoDrive(0,0,0); //stop
    }


    //calculating KP:
    //KP*(12*intotick) ~= 1
    //KP ~= 1/(# ticks in 12")
    //make KP more than that for cutoff, then cutoff before sending to autostraight
    //KP ~= 1/(12*32.357) ~= 0.0026

    public void encoderCenterTurn(double degrees, boolean counterclockwise) {
        double flTicksEnd;
        //about -9 ticks in 1 degree clockwise
        double degreeToTick = 8.8;

        double flTicksNow = Math.abs(robot.getEncoderPosition("fl"));
        if (counterclockwise) {
            flTicksEnd = degrees * degreeToTick + flTicksNow;
        } else {
            flTicksEnd = (-1) * degrees * degreeToTick + flTicksNow;
        }

        while (opModeIsActive()) {
            double KP = 0.1; //started at 0.007 from encoderstraight
            flTicksNow = Math.abs(robot.getEncoderPosition("fl"));

            double flError = flTicksEnd - flTicksNow;
            if (Math.abs(flError) < 5) {
                break;
            }
            double flPower = KP * flError;

            //cap power
            if (flPower > 1) {
                flPower = 1;
            } else if (flPower < -1) {
                flPower = -1;
            }

            robot.setMotorPower(flPower/4, flPower/4, flPower/4, flPower/4);
        }

        robot.setMotorPower(0, 0, 0, 0);
    }

     */
    /*
    public void imuTurn(double degrees, boolean counterclockwise) {


        double endYaw;
        double currentYaw = robot.getCurrentHeading();

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        currentYaw = robot.getYawDegrees(imu);

        if (counterclockwise) {
            endYaw = degrees + currentYaw;
        } else {
            endYaw = (-1) * degrees  + currentYaw;
        }

        while (opModeIsActive()) {
            double KP = 0.15; //started at 0.1 from encoderTurn
            robotOrientation = imu.getRobotYawPitchRollAngles();
            currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);

            double error = endYaw - currentYaw;

            if (Math.abs(error) < 0.5) {
                break;
            }

            double power = KP * error;

            //cap power
            if (power > 1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
            }

            robot.setMotorPower(power/4, power/4, power/4, power/4);
        }

        robot.setMotorPower(0, 0, 0, 0);
    }
    */

    public void setTurnOffYaw(IMU imu) {
        //robot.getCurrentHeading();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap, this, telemetry);

        // Reset Yaw
        //robot.resetImuYaw();

        waitForStart();

        while (opModeIsActive()) {

            robot.straightBlocking(28, true);
            sleep(100);
            robot.setHeading(0);
            sleep(100);
            robot.straightBlocking(13, false);
            sleep(100);
            robot.setHeading(0);
            sleep(100);
            robot.mecanumBlocking(17, false);
            sleep(100);
            robot.setHeading(0);
            sleep(100);
            robot.straightBlocking(38, true);
            sleep(100);
            robot.setHeading(-90);
            sleep(100);
            robot.straightBlocking(88, true);
            sleep(100);
            robot.setHeading(-90);
            sleep(100);
            robot.mecanumBlocking(25, true);
            sleep(2000);
            robot.setHeading(0);
            sleep(100);
            robot.straightBlocking(24, true);
            sleep(100);
            robot.setHeading(90);
            sleep(100);
            robot.straightBlocking(84, true);
            sleep(2000);
            robot.setHeading(-90);
            sleep(100);
            robot.straightBlocking(84, true);
            sleep(100);
            robot.setHeading(-90);
            sleep(100);
            robot.mecanumBlocking(25, true);
            sleep(100);
            break;
        }
    }
}
