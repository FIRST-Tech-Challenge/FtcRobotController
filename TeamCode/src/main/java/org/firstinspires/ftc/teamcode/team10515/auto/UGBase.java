package org.firstinspires.ftc.teamcode.team10515.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public abstract class UGBase extends LinearOpMode {


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    //public IMU imu;
    // Class Members
    private VuforiaLocalizer vuforia = null;
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    // Class Members
    private TFObjectDetector tfod;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */

    /* Declare OpMode members. */
    UGMap robot = new UGMap();
    ElapsedTime runtime = new ElapsedTime();
    Orientation angles;
    public Orientation orientation;
    //Create the angle tracker
    public double angle = 0;
    private double current_heading;

    //amount of clicks per cm
    //public final double ENCDISTANCE = 34.5781466113;

    //Next two commented lines are from RoverRuckus (working code)
    //private double wheel_diameter = 3.937;      //size of wheels original 3.75
    //public double ticks_per_inch = 52.9;//37.4; //39.68; //35.4;      //wheel_encoder_ticks / (wheel_diameter * Math.PI);

    //new calculation for Skystone Strafer Chassis
    private double wheel_diameter = 3.93700787; //inches
    public double ticks_per_inch = (383.6 * 2) / (wheel_diameter * Math.PI);      //wheel_encoder_ticks / (wheel_diameter * Math.PI);

    //private double wheel_encoder_ticks = ticks_per_inch * wheel_diameter * Math.PI;   //original 537.6

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    ElapsedTime gs_speed_timer = new ElapsedTime();
    boolean gs_first_run = true;
    int hard_stop = 5;  //seconds per operation i.e. move for this much or less

    public UGBase() {
    }

    //    public void goHorizontalLiftOut()
//    {
//        robot.AA.setPower(0.5);
//
//        while(robot.AA.getCurrentPosition() <800){
//
//        }
//        stopHorizontalLiftOut();
//    }


    public void resetEncoders() {
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(100);

        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopRobot() {
        resetEncoders();
        robot.RL.setPower(0.0);
        robot.RR.setPower(0.0);
        robot.FR.setPower(0.0);
        robot.FL.setPower(0.0);
    }


//    public void stopHorizontalLiftOut()
//    {
//        robot.AA.setPower(0);
//    }

    public double get_right_distance() {
        //return robot.rightSensor.getDistance(DistanceUnit.INCH);
        return 2;
    }

    public double get_Front_Distance() {
        //return robot.frontSensor.getDistance(DistanceUnit.INCH);
        return 2;
    }

    public double get_Back_distance() {
        //return robot.backSensor.getDistance(DistanceUnit.INCH);
        return 2;
    }

    public double get_Left_Distance() {
        //return robot.leftSensor.getDistance(DistanceUnit.INCH);
        return 2;

    }

    public void move_right_by_range(double speed, double inches_from_wall) {
        move_sideways_by_range2(90, speed, inches_from_wall);
    }

    public void move_left_by_range(double speed, double inches_from_wall) {
        move_sideways_by_range2(-90, speed, inches_from_wall);
    }

    public void move_sideways_by_range2(double heading, double speed, double inches_to_move) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;


        //resetencoders
        resetEncoders();


        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        inches = get_Left_Distance();

          telemetry.addData("Initial Distance", inches);

           telemetry.update();

           sleep(2000);

        //if (inches <= inches_to_move)
          //      destinationreached = true;


        while (opModeIsActive() && get_Left_Distance() >= inches_to_move) {
         //   telemetry.addData("Inches traveled", inches);
        //    telemetry.addData("destination reached", destinationreached);

         //   telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);

           /* inches = get_Left_Distance();

            if (inches <= inches_to_move)
                destinationreached = true;*/


        }
        stopRobot();
        sleep(50);

        telemetry.addData("Inches traveled", get_Left_Distance());
       // telemetry.addData("destination reached", destinationreached);

        telemetry.update();
        sleep(2000);

    }



    public double move_sideways_by_range(double heading, double speed, double inches_from_wall) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        //convert inches from wall to inches
        inches = get_right_distance() - inches_from_wall;
        ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down./        if (heading < 270) {
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        boolean moveLeftFlag = false;
        if (get_right_distance() <= inches_from_wall) {
            moveLeftFlag = true;
        }

        while (opModeIsActive() && !destinationreached) {
            //convert inches from wall to inches
            if (moveLeftFlag) {
                inches = inches_from_wall - get_right_distance();
            } else
                inches = get_right_distance() - inches_from_wall;

            if (!moveLeftFlag && inches <= inches_from_wall) {
                destinationreached = true;
            } else if (moveLeftFlag && inches <= 0) {
                destinationreached = true;
            }

            //if going back then
            //right distance = get right distance
            //if previous right > right distance
            //turn right by 1 degree
            //if previous right < right distance
            //turn left

            //if going forward then
            //right distance = get right distance
            //if previous right > right distance
            //turn left by 1 degree
            //if previous right < right distance
            //turn right


            telemetry.addData("Inches to travel", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);
        }

        stopRobot();
        sleep(50);

        return highest_ticks_traveled / ticks_per_inch;
    }

    public void goToFoundation(double heading, double speed, double inches_to_move)
    {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;


        //resetencoders
        resetEncoders();


        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        inches = get_right_distance();

        if (inches >= inches_to_move)
            destinationreached = true;

        while (opModeIsActive() && !destinationreached)
        {
            telemetry.addData("Inches traveled", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);

            inches = get_right_distance();
            if (inches >= inches_to_move)
                destinationreached = true;

        }
        stopRobot();
        sleep(50);

        telemetry.addData("Inches traveled", inches);
        telemetry.addData("destination reached", destinationreached);

        telemetry.update();

    }



    public double move_forward_by_range(double speed, double inches_from_wall)
    {
        return move_by_range(0, speed, inches_from_wall);
    }

    public double move_backward_by_range(double speed, double inches_from_wall)
    {
        return move_by_range(180, speed, inches_from_wall);
    }
    public void goToStone(double heading, double speed, double inches_to_move)
    {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;


        //resetencoders
        resetEncoders();


        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        inches = get_Back_distance();

        if (inches >= inches_to_move)
            destinationreached = true;

        while (opModeIsActive() && !destinationreached)
        {
            telemetry.addData("Inches traveled", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);

            inches = get_Back_distance();
            if (inches >= inches_to_move)
                destinationreached = true;

        }
        stopRobot();
        sleep(50);

        telemetry.addData("Inches traveled", inches);
        telemetry.addData("destination reached", destinationreached);

        telemetry.update();

    }
    public void goAwayFromStone(double heading, double speed, double inches_to_move)
    {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;

        //resetencoders
        resetEncoders();



        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        inches = get_Front_Distance();

        if (inches <= inches_to_move)
            destinationreached = true;

        while (opModeIsActive() && !destinationreached)
        {
            telemetry.addData("Inches traveled", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);

            inches = get_Front_Distance();
            if (inches <= inches_to_move)
                destinationreached = true;

        }
        stopRobot();
        sleep(50);

        telemetry.addData("Inches traveled", inches);
        telemetry.addData("destination reached", destinationreached);

        telemetry.update();

    }

    public void moveToFoundation(double heading, double speed, double inches_to_move)
    {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;

        resetEncoders();
        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        inches = get_Back_distance();

        if (inches <= inches_to_move)
            destinationreached = true;

        while (opModeIsActive() && !destinationreached)
        {
            telemetry.addData("Inches traveled", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);

            inches = get_Back_distance();
            if (inches <= inches_to_move)
                destinationreached = true;

        }
        stopRobot();
        sleep(50);

        telemetry.addData("Inches traveled", inches);
        telemetry.addData("destination reached", destinationreached);

        telemetry.update();

    }


    public double move_by_range(double heading, double speed, double inches_from_wall) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        double inches = 0;
        boolean destinationreached = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        //convert inches from wall to inches
        //inches = get_right_distance() - inches_from_wall;
        //ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        boolean moveBackFlag = false;
        if (get_Front_Distance() <= inches_from_wall) {
            moveBackFlag = true;
        }

        while (opModeIsActive() && !destinationreached) {
            //convert inches from wall to inches
            if (moveBackFlag) {
                inches = inches_from_wall - get_Front_Distance();
            } else
                inches = get_Front_Distance() - inches_from_wall;


            if (!moveBackFlag && inches <= inches_from_wall) {
                destinationreached = true;
            } else if (moveBackFlag && inches <= 0) {
                destinationreached = true;
            }


            telemetry.addData("Inches to travel", inches);
            telemetry.addData("destination reached", destinationreached);

            telemetry.update();

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);
        }

        stopRobot();
        sleep(50);

        return highest_ticks_traveled / ticks_per_inch;
    }
    public UGCV.numRings getRingsUsingImage(boolean red){
        //initVuforia();

        UGCV STVuforia = new UGCV(vuforia);
        UGCV.numRings position = UGCV.numRings.ZERO;

        position = STVuforia.GetPosition(true, red);
        return position;
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;
        double prev_heading = 0;
        ElapsedTime timeout_timer = new ElapsedTime();

        current_heading = get_current_heading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;
        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeout_timer.reset();
        prev_heading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && timeout_timer.seconds() < 2) {
            //wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 30, 2) + 15) / 100;
            wheel_power = (0.85 * degrees_to_turn) / 100;
            if (go_right) {
                wheel_power = -wheel_power;
            }

            robot.FR.setPower(wheel_power);
            robot.RR.setPower(wheel_power);
            robot.FL.setPower(-wheel_power);
            robot.RL.setPower(-wheel_power);

            current_heading = get_current_heading();
            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prev_heading) > 1) {
                timeout_timer.reset();
                prev_heading = current_heading;
            }
        }
        stopRobot();
    }
    public void turn_to_heading2(double target_heading, double speedModifier) {
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        currentHeading = readCurrentHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > 8 && opModeIsActive() && timeoutTimer.seconds() < 2) {  // 11/21 changed from .5 to .3

            //liftSystem.runLift();

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 2.2) ) / 100;
            } else {
                if (speedModifier != 0) {
                    wheelPower = (Math.pow((degreesToTurn) / speedModifier, 4) + 35) / 100;
                } else {
                    wheelPower = (Math.pow((degreesToTurn) / 30, 4) + 15) / 100;
                }
            }

            if (goRight) {

                double x = - wheelPower;
                wheelPower = x;

            }

            double wheelPowerAdjust = 0; //= (Math.pow(Math.abs(inchesOff) / 15, 3) + 0) / 100;
            wheelPower = 0.25;

            robot.FL.setPower(-wheelPower - wheelPowerAdjust);
            robot.FR.setPower(wheelPower - wheelPowerAdjust);
            robot.RL.setPower(-wheelPower - wheelPowerAdjust);
            robot.RR.setPower(wheelPower - wheelPowerAdjust);


            currentHeading = readCurrentHeading();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            goRight = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                goRight = !goRight;
                degreesToTurn = 360 - degreesToTurn;
            }

            if (Math.abs(currentHeading - prevHeading) > 1) {  // if it has turned at least one degree
                timeoutTimer.reset();
                prevHeading = currentHeading;
            }

        }

        stopRobot();
        telemetry.addData("Heading: ", currentHeading);
        //telemetry.addData("Odometer XInches: ", currentXInches);
        //telemetry.addData("Odometer YInches: ", currentYInches);
        telemetry.update();

    }
    public double readCurrentHeading() {
        double currentHeading;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        if (currentHeading < 0) {
            currentHeading = -currentHeading;
        } else {
            currentHeading = 360 - currentHeading;
        }
        return currentHeading;
    }

    public double get_current_heading() {
        orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = orientation.firstAngle;

        if (current_heading < 0) {
            current_heading = -current_heading;
        } else {
            current_heading = 360 - current_heading;
        }
        current_heading = shift_heading(current_heading);
        return current_heading;
    }

    private double shift_heading(double heading) {
        double shiftvalue = 3;
        heading = heading + shiftvalue;

        if (heading >= 360) {
            heading = heading - 360;
        } else if (heading < 0) {
            heading = heading + 360;
        }
        return heading;
    }


    public void turnRight(double speed, double period) {

        //  Spin right x seconds
        robot.RL.setPower(-speed);
        robot.RR.setPower(speed);
        robot.FL.setPower(-speed);
        robot.FR.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public void turnLeft(double speed, double period) {

        //  Spin Left for x seconds
        robot.RL.setPower(speed);
        robot.RR.setPower(-speed);
        robot.FL.setPower(speed);
        robot.FR.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }

    }

    public void strafeRight(double speed, double period) {

        //  Spin right x seconds
        robot.RL.setPower(-speed);
        robot.RR.setPower(speed);
        robot.FL.setPower(speed);
        robot.FR.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        robot.FL.setPower(0);
        robot.FR.setPower(0);

    }

    public void strafeLeft(double speed, double period) {

        //  Spin Left for x seconds
        robot.RL.setPower(speed);
        robot.RR.setPower(-speed);
        robot.FL.setPower(-speed);
        robot.FR.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        robot.FL.setPower(0);
        robot.FR.setPower(0);


    }

    public void goStraight(double speed)
    {

        robot.RL.setPower(speed);
        robot.RR.setPower(speed);
        robot.FL.setPower(speed);
        robot.FR.setPower(speed);

        runtime.reset();

    }

    public void moveStraightEncoder(int distance,double power) {
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FR.setTargetPosition(distance);
        robot.FL.setTargetPosition(distance);
        robot.RR.setTargetPosition(distance);
        robot.RL.setTargetPosition(distance);


        robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goStraight(power);

        while (robot.FR.isBusy() && robot.FL.isBusy() && robot.RR.isBusy() && robot.RL.isBusy()) {

        }
        stopRobot();
        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
        public void repositionBot (double angleDegrees){

            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("heading", angles.firstAngle);
            telemetry.addData("firstAngle", angles.firstAngle);
            telemetry.update();
            sleep(2000);

            while (angles.firstAngle > angleDegrees || angles.firstAngle < angleDegrees) {
                if (angles.firstAngle > angleDegrees) {
                    turnLeft(0.2, 0.1);

                } else if (angles.firstAngle < angleDegrees) {
                    turnRight(0.2, 0.1);
                }

                stopRobot();
                sleep(200);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                sleep(200);
                if (angles.firstAngle > angleDegrees - 5 && angles.firstAngle < angleDegrees + 5) {
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    sleep(100);
                    if (angles.firstAngle > angleDegrees - 8 && angles.firstAngle < angleDegrees + 8) {
                        telemetry.addData("firstAngle", angles.firstAngle);
                        telemetry.update();
                        //   sleep(1000);
                        break;
                    }

                    telemetry.addData("heading", angles.firstAngle);
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    sleep(100);
                }
            }
        }


        public void repositionBotAntiClock( double angleDegrees)
        {

            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("heading", angles.firstAngle);
            telemetry.addData("firstAngle", angles.firstAngle);
            telemetry.update();
            //sleep(2000);

            while (angles.firstAngle < angleDegrees || angles.firstAngle > -angleDegrees) {
                if (angles.firstAngle < angleDegrees) {
                    turnLeft(0.2, 0.1);

                } else if (angles.firstAngle > -angleDegrees) {
                    turnRight(0.2, 0.1);
                }

                stopRobot();
                sleep(200);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                sleep(200);
                if (angles.firstAngle > angleDegrees - 5 && angles.firstAngle < angleDegrees + 5) {
                    if (angles.firstAngle > angleDegrees - 5 && angles.firstAngle < angleDegrees + 5) {
                        telemetry.addData("firstAngle", angles.firstAngle);
                        telemetry.update();
                        //  sleep(1000);
                        break;
                    }

                    telemetry.addData("heading", angles.firstAngle);
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    sleep(100);
                }
            }

        }

    public double move_right(double speed, double inches) {
        return move_sideways(90, speed, inches);
    }

    public double move_left(double speed, double inches) {
        return move_sideways(-90, speed, inches);
    }

    public double move_forward(double speed, double inches) {
        return move_sideways(0, speed, inches);
    }

    public double move_back(double speed, double inches) {
        return move_sideways(180, speed, inches);
    }

    public void move_geneal(double backLeftPower, double frontLeftPower,
                              double backRightPower, double frontRightPower) {
        robot.RL.setPower(backLeftPower);
        robot.FL.setPower(frontLeftPower);
        robot.RR.setPower(backRightPower);
        robot.FR.setPower(frontRightPower);
    }

    public double move_sideways(double heading, double speed, double inches) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;

        boolean destinationreached = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below
        telemetry.addData("Angle Radians",angleradians);
        //telemetry.update();
        while (opModeIsActive() && !destinationreached) {

            ticks_traveled_FL = Math.abs(robot.FL.getCurrentPosition() - start_position_FL);
            ticks_traveled_RL = Math.abs(robot.RL.getCurrentPosition() - start_position_RL);
            ticks_traveled_FR = Math.abs(robot.FR.getCurrentPosition() - start_position_FR);
            ticks_traveled_RR = Math.abs(robot.RR.getCurrentPosition() - start_position_RR);

            telemetry.addData("FL Ticks", ticks_traveled_FL);
            telemetry.addData("RL Ticks", ticks_traveled_RL);
            telemetry.addData("FR Ticks", ticks_traveled_FR);
            telemetry.addData("RR Ticks", ticks_traveled_RR);
            telemetry.update();

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_FL, ticks_traveled_RL);
            highest_ticks_traveled_r = Math.max(ticks_traveled_FR, ticks_traveled_RR);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);

            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FR.setPower(rightfrontpower);
            robot.FL.setPower(leftfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);


        }

        stopRobot();
        sleep(50);

        return highest_ticks_traveled / ticks_per_inch;
    }



    public double move_back_lift(double speed, double inches) {
        return move_sideways_lift(180, speed, inches);
    }

    public double move_sideways_lift(double heading, double speed, double inches) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;

        boolean destinationreached = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !destinationreached) {

            ticks_traveled_FL = Math.abs(robot.FL.getCurrentPosition() - start_position_FL);
            ticks_traveled_RL = Math.abs(robot.RL.getCurrentPosition() - start_position_RL);
            ticks_traveled_FR = Math.abs(robot.FR.getCurrentPosition() - start_position_FR);
            ticks_traveled_RR = Math.abs(robot.RR.getCurrentPosition() - start_position_RR);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_FL, ticks_traveled_RL);
            highest_ticks_traveled_r = Math.max(ticks_traveled_FR, ticks_traveled_RR);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);


            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            leftfrontpower = speed * Math.cos(angleradians);
            rightfrontpower = speed * Math.sin(angleradians);
            leftrearpower = speed * Math.sin(angleradians);
            rightrearpower = speed * Math.cos(angleradians);

            robot.FL.setPower(leftfrontpower);
            robot.FR.setPower(rightfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);

//            if (robot.AA.getCurrentPosition() <300){
//                robot.AA.setPower(0.3);
//            }else{
//                robot.AA.setPower(0);
//            }


        }

        stopRobot();
        sleep(50);

        return highest_ticks_traveled / ticks_per_inch;
    }

    public double move_sideways_rampup(double heading, double speed, double inches, double rampDownInches, boolean armOut) {
        double angleradians;

        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;

        double actualSpeed = 0.1;

        int rampdown_ticks;

        boolean destinationreached = false;

        boolean rampdown = false;

        int ticks_to_travel;
        int start_position_FL;
        int start_position_RL;
        int start_position_FR;
        int start_position_RR;
        int ticks_traveled_FL;
        int ticks_traveled_RL;
        int ticks_traveled_FR;
        int ticks_traveled_RR;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        //resetencoders
        resetEncoders();

        start_position_FL = robot.FL.getCurrentPosition();
        start_position_RL = robot.RL.getCurrentPosition();
        start_position_FR = robot.FR.getCurrentPosition();
        start_position_RR = robot.RR.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticks_per_inch);

        rampdown_ticks = (int) (rampDownInches * ticks_per_inch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (heading < 270) {
            angleradians = ((heading - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - heading) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !destinationreached) {

            ticks_traveled_FL = Math.abs(robot.FL.getCurrentPosition() - start_position_FL);
            ticks_traveled_RL = Math.abs(robot.RL.getCurrentPosition() - start_position_RL);
            ticks_traveled_FR = Math.abs(robot.FR.getCurrentPosition() - start_position_FR);
            ticks_traveled_RR = Math.abs(robot.RR.getCurrentPosition() - start_position_RR);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_FL, ticks_traveled_RL);
            highest_ticks_traveled_r = Math.max(ticks_traveled_FR, ticks_traveled_RR);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);


            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            if (highest_ticks_traveled >= rampdown_ticks) {
                rampdown = true;
            }

            if (actualSpeed < speed && !rampdown) {
                actualSpeed += 0.05;
            }

            if (rampdown) {
                if (actualSpeed > 0.3) {
                    actualSpeed -= 0.01;
                } else if (actualSpeed < 0.3){ // not sure why we need this but once actual speed went below 0.3 {
                    actualSpeed = 0.3;
                }

//                if (armOut) {
//                    armOut();
//                    stopIntake();
//                    armOut = false;
//                }
            }

            telemetry.addData("actualspeed", actualSpeed);
            telemetry.update();
            leftfrontpower = actualSpeed * Math.cos(angleradians);
            rightfrontpower = actualSpeed * Math.sin(angleradians);
            leftrearpower = actualSpeed * Math.sin(angleradians);
            rightrearpower = actualSpeed * Math.cos(angleradians);


            robot.FR.setPower(rightfrontpower);
            robot.FL.setPower(leftfrontpower);
            robot.RL.setPower(leftrearpower);
            robot.RR.setPower(rightrearpower);


        }
        stopRobot();
        return highest_ticks_traveled / ticks_per_inch;
    }

}