
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("ALL")
    @Autonomous(name="JMAC Auto", group="auto")
//@Disabled
public class JmacAuto extends LinearOpMode {

    DrivetrainHardware mDrive = new DrivetrainHardware();

    BNO055IMU imu;
    //ElapsedTime clock = new ElapsedTime();

    int ringCount;
    double globalAngle;
    Orientation lastAngles = new Orientation();

    public final double WHEEL_DIAMETER = 4.0; //Wheel diameter in inches
    public final int MOTOR_GEAR_TEETH = 1; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 15; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 28; //bruh


    @Override
    public void runOpMode() throws InterruptedException {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        mDrive.init(hardwareMap);
        Vision vision = new Vision(this);

        while (!isStarted()) {
            mDrive.claw.setPosition(0);
            ringCount = 4; // vision.ringCount('r');
            telemetry.addData("Ring Count: ", ringCount);
            telemetry.update();
        }

        waitForStart();
        if (!isStopRequested()) {

            switch (ringCount)
            {
                case 0:
                    linearMovement(58, 3.5, 0.000475, 0.000058, 0.000040);
                    turnDegree(-60, 3, 0.0118,0.005, 0.0015);
                    mDrive.Arm.setPower(-0.5);
                    sleep(350);
                    mDrive.Arm.setPower(0);
                    sleep(200);
                    mDrive.claw.setPosition(1);
                    sleep(300);
                    mDrive.Arm.setPower(0.5);
                    sleep(200);
                    mDrive.claw.setPosition(0);
                    sleep(200);
                    mDrive.Arm.setPower(0);
                    turnDegree(60, 3, 0.0118,0.005, 0.0015);
                    monkeShoot();
                    linearMovement(-43.5, 4, 0.000485, 0.00008, 0.0000365);
                    turnDegree(90, 3.5, 0.0118,0.005, 0.0015);
                    mDrive.Arm.setPower(-0.5);
                    sleep(200);
                    mDrive.claw.setPosition(1);
                    sleep(200);
                    mDrive.Arm.setPower(0);
                    linearMovement(14, 2, 0.0008,0.00008, 0.0000365);
                    mDrive.claw.setPosition(0);
                    sleep(500);
                    mDrive.Arm.setPower(0.5);
                    sleep(400);
                    mDrive.Arm.setPower(0);
                    linearMovement(-15, 2,0.0008,0.00008, 0.0000385);
                    turnDegree(-90, 3.5, 0.0118,0.005, 0.0015);
                    linearMovement(43, 3.5, 0.000485, 0.000065, 0.0000365);
                    mDrive.Arm.setPower(-0.5);
                    sleep(350);
                    mDrive.Arm.setPower(0);
                    turnDegree(-75, 3, 0.0120,0.005, 0.0015);
                    mDrive.claw.setPosition(1);
                    strafeLeft();
                    break;



                case 1:
                    mDrive.claw.setPosition(1);
                    linearMovement(58, 5, 0.0004,0.00007, 0.000068);
                    turnDegree(26,5, 0.0118,0.005, 0.002);

                    mDrive.ringHopper.setPosition(1);
                    sleep(500);
                    mDrive.Intake.setPower(-0.7);
                    mDrive.ringHopper.setPosition(0.5);
                    mDrive.FlyWheel1.setPower(1);
                    mDrive.FlyWheel2.setPower(1);
                    sleep(1000);
                    mDrive.Intake.setPower(0);
                    mDrive.ringHopper.setPosition(1);
                    sleep(1000); //first shot
                    mDrive.ringHopper.setPosition(0.5);
                    turn1();
                    sleep(1000);
                    mDrive.ringHopper.setPosition(1);
                    sleep(500); //second shot
                    mDrive.ringHopper.setPosition(0.5);
                    turn2();
                    sleep(1000);
                    mDrive.ringHopper.setPosition(1);
                    sleep(500); //third shot
                    mDrive.ringHopper.setPosition(0.5);
                    mDrive.FlyWheel1.setPower(0);
                    mDrive.FlyWheel2.setPower(0);
                    mDrive.Intake.setPower(0.7);
                    sleep(750);
                    mDrive.Intake.setPower(0);

                    turnDegree(-28,5, 0.0118,0.005, 0.002);
                    linearMovement(34, 5, 0.0004,0.00007, 0.000068);
                    turnDegree(45,5, 0.0118,0.005, 0.002);
                    mDrive.Arm.setPower(-0.75);
                    sleep(750);
                    mDrive.claw.setPosition(0);
                    sleep(500);
                    mDrive.Arm.setPower(0.5);
                    sleep(750);
                    mDrive.Arm.setPower(0);
                    turnDegree(-45,5, 0.0118,0.005, 0.002);
                    linearMovement(-18, 4, 0.0004,0.00007, 0.000068);
                    break;
                case 4:

                    linearMovement(98, 3.5, 0.00042,0.00003, 0.0000475);
                    turnDegree(-60, 2.5, 0.0118,0.005, 0.0015);
                    mDrive.Arm.setPower(-0.5);
                    sleep(350);
                    mDrive.Arm.setPower(0);
                    sleep(200);
                    mDrive.claw.setPosition(1);
                    sleep(200);
                    mDrive.Arm.setPower(0.5);
                    sleep(200);
                    mDrive.claw.setPosition(0);
                    sleep(200);
                    mDrive.Arm.setPower(0);
                    turnDegree(60, 3, 0.0118,0.005, 0.0015);
                    linearMovement(-41, 3, 0.000485, 0.00008, 0.0000375);
                    monkeShoot();
                    linearMovement(-45, 3.5, 0.000485, 0.00008, 0.0000375);
                    turnDegree(90, 3.5, 0.0118,0.005, 0.0015);
                    mDrive.Arm.setPower(-0.5);
                    sleep(200);
                    mDrive.claw.setPosition(1);
                    sleep(200);
                    mDrive.Arm.setPower(0);
                    go5();
                    //linearMovement(8, 2, 0.002,0.0002, 0.0000365);
                    mDrive.claw.setPosition(0);
                    sleep(200);
                    mDrive.Arm.setPower(0.5);
                    sleep(400);
                    mDrive.Arm.setPower(0);
                    linearMovement(-20, 1.5,0.001,0.0002, 0.0000385);
                    turnDegree(-90, 3.2, 0.0118,0.005, 0.0015);
                    linearMovement(90, 2.75, 0.00042, 0.00003, 0.000045);
                    mDrive.Arm.setPower(-0.5);
                    sleep(300);
                    mDrive.Arm.setPower(0);
                    turnDegree(-80, 2, 0.015,0.005, 0.0015);
                    mDrive.claw.setPosition(1);
                    backOff();
                    break;

            }
        }
        mDrive.freeze();
    }



    /*
    public double sigmoid(double error, double ceiling, double floor, double half, double stiff) {
        return floor + (ceiling - floor) / (1 + Math.pow(Math.E, stiff * (half - error)));
    }*/

    public void linearMovement(double distance, double timeframe, double kP, double kI, double kD) {
        double conversionIndex = 1104.04; // ticks per inch
        double timeFrame = timeframe; //distance * distanceTimeIndex;
        double errorMargin = 5;
        double powerFloor = 0;
        double powerCeiling = 1;

        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        mDrive.resetEncoders();

        double targetTick = -1 * distance * conversionIndex;
        telemetry.addData("target tick", targetTick);
        telemetry.update();


        double error = targetTick;
        double errorPrev = error;
        double time = clock.seconds();
        double timePrev = time;

        double  p, d, output;
        double i = 0;

        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive()) {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            errorPrev = error;
            timePrev = time;

            double tempAvg = targetTick > 0 ? mDrive.getEncoderAvg() : -mDrive.getEncoderAvg();

            error = targetTick - tempAvg;
            time = clock.seconds();
            //telemetry.addData("error", error);
            //telemetry.addData("time", time);

            p = Math.abs(error) / 33.0 * kP;
            i += (time - timePrev) * Math.abs(error) / 33.0 * kI;
            d = Math.abs((error - errorPrev) / (time - timePrev) / 33.0 * kD);

            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);


            output = p + i - d;
            telemetry.addData("output", output);
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (error < 0) output *= -1;

            double currentAngle = imu.getAngularOrientation().firstAngle;
            double raw = globalAngle - currentAngle;
            if (raw > 180)
                raw -= 360;
            if (raw < -180)
                raw += 360;
            double fudgeFactor = 1.0 - raw / 40.0;

            if (distance > 0)
            {
                mDrive.FL.setPower(fudgeFactor * output);
                mDrive.FR.setPower(output);
                mDrive.BL.setPower(fudgeFactor * output);
                mDrive.BR.setPower(output);
            }
            else
            {
                mDrive.FL.setPower(output);
                mDrive.FR.setPower(fudgeFactor * output);
                mDrive.BL.setPower(output);
                mDrive.BR.setPower(fudgeFactor * output);
            }


            telemetry.addData("error", error);
            telemetry.update();
        }
        mDrive.freeze();
        telemetry.speak("movement took" + clock.seconds() + " seconds.");
        telemetry.update();

    }



    public void turnDegree(double degree, double timeframe, double kP, double kI, double kD)
    {
        telemetry.addLine("made it");
        telemetry.update();
        lastAngles = imu.getAngularOrientation();
        double currentAngle = lastAngles.firstAngle;
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        globalAngle += degree;
        if (globalAngle > 180)
            globalAngle -= 360;
        if (globalAngle < -180)
            globalAngle += 360;
        double leftPower, rightPower;

        // restart imu movement tracking.
        //resetAngle();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // set power to rotate.

        // rotate until turn is completed.

        double error = globalAngle - currentAngle;
        double errorPrev = error;

        double time = clock.seconds();
        double timePrev = time;

        double p, d, output;
        double i = 0;

        while (clock.seconds() < timeframe && Math.abs(error) > 1 && opModeIsActive()) {
            lastAngles = imu.getAngularOrientation();
            currentAngle = lastAngles.firstAngle;

            timePrev = time;
            errorPrev = error;

            time = clock.seconds();
            error = globalAngle - currentAngle;



            if (error > 180)
                error -= 360;
            if (error < -180)
                error += 360;

            p = Math.abs(error) * kP;
            i += (time - timePrev) * Math.abs(error) * kI;
            d = ((Math.abs(error) - Math.abs(errorPrev)) / (time - timePrev)) * kD;

            output = p + i + d;



            //telemetry.addData("output ", output);
            telemetry.addData("globalAngle", globalAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("error ", error);
            //telemetry.addData("p", p);
            //telemetry.addData("i", i);
            //telemetry.addData("d", d);*/
            telemetry.update();


            if (error > 0)
            {
                mDrive.FL.setPower(output);
                mDrive.BL.setPower(output);
                mDrive.FR.setPower(-output);
                mDrive.BR.setPower(-output);
            }
            else
            {
                mDrive.FL.setPower(-output);
                mDrive.BL.setPower(-output);
                mDrive.FR.setPower(output);
                mDrive.BR.setPower(output);
            }
        }
        mDrive.freeze();
    }


    public void turn45()
    {
        mDrive.FL.setPower(-0.4);
        mDrive.BL.setPower(-0.4);
        mDrive.FR.setPower(0.4);
        mDrive.BR.setPower(0.4);
        sleep(600);
        mDrive.FL.setPower(0);
        mDrive.BL.setPower(0);
        mDrive.FR.setPower(0);
        mDrive.BR.setPower(0);
    }

    public void shoot()
    {
        mDrive.ringHopper.setPosition(1);
        sleep(400);
        mDrive.Intake.setPower(-0.75);
        mDrive.ringHopper.setPosition(0.5);
        mDrive.FlyWheel1.setPower(1);
        mDrive.FlyWheel2.setPower(1);
        sleep(400);
        mDrive.Intake.setPower(-0.4);
        mDrive.ringHopper.setPosition(1);
        sleep(700); //first shot
        mDrive.ringHopper.setPosition(0.5);
        sleep(750);
        mDrive.ringHopper.setPosition(1);
        sleep(700); //second shot
        mDrive.ringHopper.setPosition(0.5);
        sleep(750);
        mDrive.ringHopper.setPosition(1);
        sleep(450); //third shot
        mDrive.ringHopper.setPosition(0.5);
        mDrive.FlyWheel1.setPower(0);
        mDrive.FlyWheel2.setPower(0);
        mDrive.Intake.setPower(0);
    }

    public void resetShooter()
    {
        mDrive.ringHopper.setPosition(0);
        mDrive.Intake.setPower(0.7);
        sleep(750);
        mDrive.Intake.setPower(0);
        mDrive.ringHopper.setPosition(0.5);
    }

    public void strafeLeft()
    {
        mDrive.FL.setPower(1);
        mDrive.BL.setPower(-1);
        mDrive.FR.setPower(-1);
        mDrive.BR.setPower(1);
        sleep(500);
        mDrive.FL.setPower(0);
        mDrive.BL.setPower(0);
        mDrive.FR.setPower(0);
        mDrive.BR.setPower(0);
    }

    public void backOff()
    {
        mDrive.FL.setPower(1);
        mDrive.BL.setPower(1);
        mDrive.FR.setPower(0.5);
        mDrive.BR.setPower(0.5);
        sleep(900);
        mDrive.FL.setPower(0);
        mDrive.BL.setPower(0);
        mDrive.FR.setPower(0);
        mDrive.BR.setPower(0);
    }

    public void go5()
    {
        mDrive.FL.setPower(-0.4);
        mDrive.BL.setPower(-0.4);
        mDrive.FR.setPower(-0.4);
        mDrive.BR.setPower(-0.4);
        sleep(200);
        mDrive.FL.setPower(0);
        mDrive.BL.setPower(0);
        mDrive.FR.setPower(0);
        mDrive.BR.setPower(0);
        sleep(500);
    }

    public void turn1()
    {
        mDrive.FL.setPower(0.3);
        mDrive.BL.setPower(0.3);
        mDrive.FR.setPower(-0.3);
        mDrive.BR.setPower(-0.3);
        sleep(110);
        mDrive.FL.setPower(0);
        mDrive.BL.setPower(0);
        mDrive.FR.setPower(0);
        mDrive.BR.setPower(0);
    }

    public void turn2()
    {
        mDrive.FL.setPower(0.3);
        mDrive.BL.setPower(0.3);
        mDrive.FR.setPower(-0.3);
        mDrive.BR.setPower(-0.3);
        sleep(90);
        mDrive.FL.setPower(0);
        mDrive.BL.setPower(0);
        mDrive.FR.setPower(0);
        mDrive.BR.setPower(0);
    }

    public void monkeShoot()
    {
        mDrive.FlyWheel1.setPower(0.9);
        mDrive.FlyWheel2.setPower(0.9);
        sleep(1500);
        mDrive.ringHopper.setPosition(1);
        sleep(500);
        mDrive.ringHopper.setPosition(0.5);
        sleep(750);
        mDrive.ringHopper.setPosition(1);
        sleep(500);
        mDrive.ringHopper.setPosition(0.5);
        sleep(750);
        mDrive.ringHopper.setPosition(1);
        sleep(500);
        mDrive.ringHopper.setPosition(0.5);
        sleep(750);
        mDrive.ringHopper.setPosition(1);
        sleep(500);
        mDrive.ringHopper.setPosition(0.5);
        mDrive.FlyWheel1.setPower(0);
        mDrive.FlyWheel2.setPower(0);
    }
}