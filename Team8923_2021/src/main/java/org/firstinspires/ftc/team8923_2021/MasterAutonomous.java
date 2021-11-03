package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team8923_2021.Constants;
import org.firstinspires.ftc.team8923_2021.MasterOpMode;

@Autonomous(name="MasterAutonomous")
public abstract class MasterAutonomous extends MasterOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double robotX;
    double robotY;
    double robotAngle;
    double headingOffset = 0.0;

    int newTargetLeft;
    int newTargetRight;

    int errorLeft;
    int errorRight;

    double speedLeft;
    double speedRight;

    double Kmove = 1.0f/1200.0f;

    int TOL = 100;

    boolean isDoneSettingUp = false;

    //Used to calculate distance traveled between loops
    int lastEncoderLeft = 0;
    int lastEncoderRight = 0;

    boolean autoReverseDrive = false;

    Alliance alliance = Alliance.BLUE;
    Destinations destination = Destinations.SQUAREA;

    int element;

    int delays = 0;
    int numOfSecondsDelay = 0;
    int delayTime = 0;

    double DRIVE_POWER_CONSTANT = 1.0/1000;
    double TURN_POWER_CONSTANT = 1.0/65;

    double MIN_DRIVE_POWER = 0.2;

    enum Alliance{
        BLUE,RED
    }

    enum Destinations{
        SQUAREA, SQUAREB, SQUAREC
    }

    public void initAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initHardware();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Init State", "Init Finished");
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Delay Time", delayTime);

        //Set last know encoder values
        lastEncoderRight = motorLeft.getCurrentPosition();

        //set IMU heading offset
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        telemetry.clear();
        telemetry.update();
        telemetry.addLine("Initialized. Ready to start!");
    }

    public void configureAutonomous(){
        while(!isDoneSettingUp){
            if(gamepad1.x){
                alliance = Alliance.BLUE;
            }else if(gamepad1.b){
                alliance = Alliance.RED;
            }
            if(gamepad1.dpad_up){
                delays++;
            }else if(gamepad1.dpad_down){
                delays--;
            }
            if(element == 0){
                destination = Destinations.SQUAREA;

            }else if(element == 1){
                destination = Destinations.SQUAREB;
            }else if(element == 4){
                destination = Destinations.SQUAREC;
            }

            if(gamepad1.start){
                isDoneSettingUp = true;
            }
            // input information
            telemetry.addLine("Alliance Blue/Red: X/B");
            telemetry.addLine("Add a delay: D-Pad Up/Down");
            telemetry.addLine("toggle objective: a park/park and foundation");
            telemetry.addLine("After routine is complete and robot is on field, press Start");

            telemetry.addLine();
            telemetry.addData("alliance: ", alliance);
            telemetry.addData("delays:", delays);
            telemetry.addData("destination", destination);
            telemetry.update();
        }
    }

    //using imu
    public void imuPivot(double referenceAngle, double targetAngle, double maxSpeed, double kAngle, double timeout){
        runtime.reset();
        //counter-clockwise is positive
        double pivot;
        double currentRobotAngle;
        double angleError;

        targetAngle = referenceAngle + targetAngle;
        targetAngle = adjustAngles(targetAngle);
        do{
            currentRobotAngle = imu.getAngularOrientation().firstAngle;
            angleError =  currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0){
                pivot = Range.clip(pivot, 0.15, maxSpeed);
            }else{
                pivot = Range.clip(pivot, -maxSpeed, -0.15);
            }

            speedLeft = pivot;
            speedRight = -pivot;

            motorLeft.setPower(speedLeft);
            motorRight.setPower(speedRight);

            idle();
        }
        while((opModeIsActive() && (Math.abs(angleError) > 3.0)) && (runtime.seconds() < timeout));
        stopDriving();
    }

    public void reverseImuPivot(double referenceAngle, double targetAngle, double maxSpeed, double kAngle, double timeout){
        runtime.reset();
        //counter-clockwise is positive
        double pivot;
        double currentRobotAngle;
        double angleError;

        targetAngle = referenceAngle + targetAngle;
        targetAngle = adjustAngles(targetAngle);
        do{
            currentRobotAngle = imu.getAngularOrientation().firstAngle;
            targetAngle = adjustAngles(targetAngle);
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if(pivot >= 0.0){
                pivot = Range.clip(pivot, 0.15, maxSpeed);
            }else{
                pivot = Range.clip(pivot, -maxSpeed, -0.15);
            }

            speedLeft = -pivot;
            speedRight = pivot;

            motorLeft.setPower(speedLeft);
            motorRight.setPower(speedRight);
            idle();
        } while(opModeIsActive() && (Math.abs(angleError) > 3.0) && (runtime.seconds() < timeout));

        stopDriving();
    }

    public void moveForward(int distance, double speed, double minSpeed) {
        //sets a target position to drive to
        newTargetLeft = motorLeft.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_INCH * distance);
        newTargetRight = motorRight.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_INCH * distance);

        motorLeft.setTargetPosition(newTargetLeft);
        motorRight.setTargetPosition(newTargetRight);

        //set to RUN_TO_POSITION
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //sets the speed
        motorLeft.setPower(speed);
        motorRight.setPower(speed);

        // loop until both motors are not busy, then stop.
        while (motorLeft.isBusy() && motorRight.isBusy()) {
            telemetry.addData("Path1",  "Running to %7d :%7d", newTargetLeft,  newTargetRight);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    motorLeft.getCurrentPosition(),
                    motorRight.getCurrentPosition());
            telemetry.update();
        }

        motorRight.setPower(0);
        motorLeft.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void sendTelemetry(){

        //Informs drivers of robot location
        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Robot Angle", imu.getAngularOrientation().firstAngle);
    }

    private void stopDriving() {
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
    }

    //normalizing the angle to be between -180 to 180
    private double adjustAngles(double angle){
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }

    private double normalizeAngle(double rawAngle){
        while(Math.abs(rawAngle) > 180){
            rawAngle -= Math.signum(rawAngle) * 360;
        }
        return rawAngle;
    }

    private void updateRobotLocation(){
        // Update robot angle
        // subtraction here b/c imu returns a negative rotation when turned to the right
        robotAngle = headingOffset - imu.getAngularOrientation().firstAngle;

        // Take average of encoder ticks to find translational x and y components. Right and BL are
        // negative because of the direction at which they turn when going sideways
        int deltaLeft = motorLeft.getCurrentPosition() - lastEncoderLeft;
        int deltaRight = motorRight.getCurrentPosition() - lastEncoderRight;
        double deltaX = (deltaLeft - deltaRight - deltaLeft + deltaRight) / 4.0;
        double deltaY = (deltaLeft + deltaRight + deltaLeft + deltaRight) / 4.0;

        telemetry.addData("deltaX", deltaX);
        telemetry.addData("deltaY", deltaY);

        // Convert to mm
        //TODO: maybe wrong proportions? something about 70/30 effectiveness maybe translation to MM is wrong
        deltaX *= Constants.TICKS_PER_INCH;
        deltaY *= Constants.TICKS_PER_INCH;

    /*
     * Delta x and y are intrinsic to the robot, so they need to be converted to extrinsic.
     * Each intrinsic component has 2 extrinsic components, which are added to find the
     * total extrinsic components of displacement. The extrinsic displacement components
     * are then added to the previous position to set the new coordinates
     */

        robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
        robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));

        // Set last encoder values for next loop

        lastEncoderLeft = motorLeft.getCurrentPosition();
        lastEncoderRight = motorRight.getCurrentPosition();


    }
}






