package org.firstinspires.team8923_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

abstract public class MasterAutonomous extends MasterOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    double robotX;
    double robotY;
    double robotAngle;
    double headingOffset = 0.0;

    int newTargetFL;
    int newTargetFR;
    int newTargetBL;
    int newTargetBR;

    int errorFR;
    int errorFL;
    int errorBR;
    int errorBL;

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    double Kmove = 1.0f/1200.0f;


    int TOL = 100;

    boolean isDoneSettingUp = false;

    //Used to calculate distance traveled between loops
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    boolean autoReverseDrive = false;

    Alliance alliance = Alliance.BLUE;
    Destinations destination = Destinations.SQUAREA;

    int rings;



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
        lastEncoderFR = motorLeft.getCurrentPosition();
        lastEncoderBL = motorRight.getCurrentPosition();


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
            if(rings == 0){
                destination = Destinations.SQUAREA;

            }else if(rings == 1){
                destination = Destinations.SQUAREB;
            }else if(rings == 4){
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

    public void moveAuto(double x, double y, double speed, double minSpeed) throws InterruptedException {

        newTargetFR = motorLeft.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_INCH * y) + (int) Math.round(Constants.TICKS_PER_INCH * x * 1.15);
        newTargetBR = motorRight.getCurrentPosition() - (int) Math.round(Constants.TICKS_PER_INCH * y) - (int) Math.round(Constants.TICKS_PER_INCH * x * 1.15);

        do {

            errorFL = newTargetFL - motorLeft.getCurrentPosition();
            speedFL = Math.abs(errorFL * Kmove);
            speedFL = Range.clip(speedFL, minSpeed, speed);
            speedFL = (speedFL * Math.signum(errorFL));

            errorFR = newTargetFR - motorRight.getCurrentPosition();
            speedFR = Math.abs(errorFR * Kmove);
            speedFR = Range.clip(speedFR, minSpeed, speed);
            speedFR = (speedFR * Math.signum(errorFR));

            motorLeft.setPower(speedFL);
            motorRight.setPower(speedFR);

            idle();
        }
        while (opModeIsActive() && Math.abs(errorFL) > TOL || Math.abs(errorFR) > TOL || Math.abs(errorBR) > TOL || Math.abs(errorBL) > TOL);
        stopDriving();
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

            speedFL = pivot;
            speedFR = pivot;
            speedBL = pivot;
            speedBR = pivot;

            motorLeft.setPower(speedFL);
            motorRight.setPower(speedFR);

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

            speedFL = -pivot;
            speedFR = pivot;
            speedBL = -pivot;
            speedBR = pivot;


            motorLeft.setPower(speedBL);
            motorRight.setPower(speedBR);
            idle();
        } while(opModeIsActive() && (Math.abs(angleError) > 3.0) && (runtime.seconds() < timeout));

        stopDriving();

    }

    public void sendTelemetry(){

        //Informs drivers of robot location
        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Robot Angle", imu.getAngularOrientation().firstAngle);
    }

    private void stopDriving(){


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

        // Calculate how far each motor has turned since last time

        int deltaBL = motorLeft.getCurrentPosition() - lastEncoderBL;
        int deltaBR = motorRight.getCurrentPosition() - lastEncoderBR;

        // Take average of encoder ticks to find translational x and y components. FR and BL are
        // negative because of the direction at which they turn when going sideways
        int deltaFL = motorLeft.getCurrentPosition() - lastEncoderFL;
        int deltaFR = motorRight.getCurrentPosition() - lastEncoderFR;
        double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4.0;
        double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4.0;

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

        lastEncoderBL = motorLeft.getCurrentPosition();
        lastEncoderBR = motorRight.getCurrentPosition();


    }

    public void runIntake() throws InterruptedException{
        //intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //intakeLeft.setPower(Constants.INTAKE_PWR);
        //intakeLeft.setPower(Constants.INTAKE_PWR);
    }

    public void turnOffIntake() throws InterruptedException{
        //intakeLeft.setPower(0);
        //intakeRight.setPower(0);
    }







    public void moveBackAndIntake() throws InterruptedException{
        //runIntake();
        moveAuto(0, -12, 1, 0.3);
        sleep(900);
    }

    public void moveForwardAndIntake() throws InterruptedException{
        //runIntake();
        moveAuto(0, 12, 1, 0.3);
        sleep(900);
    }






}






