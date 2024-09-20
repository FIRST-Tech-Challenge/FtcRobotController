package org.firstinspires.ftc.teamcode.controller;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.IMU.IMUControl;
import org.firstinspires.ftc.teamcode.util.Logging;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.ImuHardware;

import java.io.IOException;

/**
 * MecanumSynchronousDriver serves as an interface to the drive motors giving the user directional
 * methods to command the bot.  The movements are backed up by PID controller to ensure the bot
 * stays on course.  This class is intended to be used by autonomous programs.
 */
public class MecanumSynchronousDriver<imuControl> extends MechanicalDriveBase
{

   /**
    * Number of ticks covered in one inch of movement for the odometery wheel.
    * NOTE: value may need fine tuning.
    */
   private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303
//                    Circumference of the robot when turning == encoder dist from center (radius) * 2 * pi
//                                        ^^^
//   private final double ticksPerDegree = (ticksPerInch * 64.09) / 360; // 16,000
 //  private final double ticksPerDegree = (ticksPerInch * 56.0) / 360; // 16,000
   //private final double ticksPerDegree = (ticksPerInch * 54.6) / 360; // 16,000
//private final double ticksPerDegree = (ticksPerInch * 55.0) / 360;
   //private final double ticksPerDegree = (ticksPerInch * 54.0) / 360;  // 195.57536208817444
//private final double ticksPerDegree = 202.75;//190;  //FOR BATT 12.3 VOLTS
   private final double ticksPerDegree = 199;  // 12.63 V
//68,466 for 360
//34,233 for 180
//17,116 for 90
// 8,558 for 45

   /**
    * Logging method used to write data to file.
    * NOTE: This seems to cause the dev to have to hard reset the bot after each run, making us of
    * this class not ideal.  Need to debug cause.
    */
   Logging logger;

   /**
    * Local copy of the opMode.
    */
   LinearOpMode mOpMode;

   /**
    * PD controller to correct the turning of the bot while driving a straight line.
    */
   PIDController pidDrive;

   /**
    * PID controller to correct the lateral movement (strafing) while the bot is driving a straight line.
    */
   PIDController pidStrafe;


   PIDController pidRotateImu;

   PIDController pidRotateOd;

   double rotation;

   //Time out timer to kep the robot from getting stuck
   ElapsedTime time;

    /**
     * Constructor for MechanicalDriveBase from the hardware map
     *
     * @param hardwareMap the hardware map
     */
    public MecanumSynchronousDriver(HardwareMap hardwareMap, LinearOpMode opMode) throws IOException
    {
       super(opMode.hardwareMap);

       time = new ElapsedTime();

       //logger.setup();
       logger.log("Starting MecanumSynchronousDriver");

       mOpMode = opMode;
       opMode.telemetry.update();

       //The input can be a large value, therefore the values of Kp needs to be small to compensate.
       //
       // Set PID proportional value to produce non-zero correction value when robot veers off
       // straight line. P value controls how sensitive the correction is.
       pidDrive = new PIDController(0.01, 0, 0.001);

       pidStrafe = new PIDController(0.00001, 0, 0);

       // Set PID proportional value to start reducing power at about 50 degrees of rotation.
       // P by itself may stall before turn completed so we add a bit of I (integral) which
       // causes the PID controller to gently increase power if the turn is not completed.
       pidRotateImu = new PIDController(.02, .003, .001);

       pidRotateOd = new PIDController(.0003, .0000002, .000);  // 180 ok... kinda

//       Logging.setup();
       Logging.log("Starting MecanumSynchronousDriver Logging");
    }

   /**
    * Drives the bot forward or backward in a straight line.
    * @param target distance in inches to travel.
    * @param forward indicates direction of travel.  -1 is forward 1 is backwards?
    * @param speed double value indicating the speed from 0 to 1.
    * @param seconds Will stop running to position if time exceeds this time
    */
    public void forward(double target, int forward, double speed, int seconds)
    {
        this.resetEncoders();
        resetRunMode();

        time.reset();
        time.startTime();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(0, 500);
        pidDrive.setTolerance(1);
//       pidDrive.setContinuous();
        pidDrive.enable();

        // Set up parameters for strafe correction.
        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, 0.3);
        pidStrafe.setInputRange(-5000, 5000);
        pidStrafe.enable();

        speed *= forward;
        int leftFrontPos = this.lf.getCurrentPosition();

        if (forward == 1)
            leftFrontPos += target * ticksPerInch;
        else
            leftFrontPos -= target * ticksPerInch;

        while (mOpMode.opModeIsActive() && time.seconds() < seconds)
        {
            int frontLeftPos, frontRightPos;

            frontLeftPos = this.lf.getCurrentPosition();
            frontRightPos = this.rf.getCurrentPosition();

            double currPosTicks = (frontLeftPos + frontRightPos) / 2;


            //if the number is positive the bot is slipping right
            //if the number is negative the bot is slipping left
            //lf and rf are added because rf is reverse of lf direction.
            int wheelDifference = frontLeftPos - frontRightPos;

            // if the number is positive the bot strafed left
            // if the number is negative the bot strafed right
            int strafeDifference = this.rb.getCurrentPosition();

            // Use PID with imu input to drive in a straight line.
            // pos is right turn, neg is left turn
            double correction = pidDrive.performPID(wheelDifference);
            mOpMode.telemetry.addData("correction ", "correction: " + correction + " wheelDif: " + wheelDifference);

            double strafeCorrection = pidStrafe.performPID(strafeDifference);
            //mOpMode.telemetry.addData("strafeCorrection ", "correction: " + strafeCorrection + " strafeDifference: " + strafeDifference);

            //Logging.log("left Encoder = %d, Right Encoder = %d wheelDifference = %d correction = %f currPosTicks = %f", this.lf.getCurrentPosition(), this.rf.getCurrentPosition(), wheelDifference, correction,currPosTicks);
            Logging.log("left Encoder = %d, Right Encoder = %d wheelDifference = %d" , frontLeftPos, frontRightPos, wheelDifference);

            //this.driveMotors(speed, 0, 0, 1); //run with no PID

            if (pidDrive.onTarget())
            {
                correction = 0;
            }
            else
            {
                correction = correction * (speed * 0.2);
            }
            this.driveMotors(speed, (correction * forward) + 0.01, -strafeCorrection, 1); // run with PID

//               this.driveMotors(speed, 0, 0, 1);

//            mOpMode.telemetry.addData("Encoder", "left: " + lf.getCurrentPosition() + " right: " + rf.getCurrentPosition() + " strafe: " + rb.getCurrentPosition());
//            mOpMode.telemetry.update();

            Logging.log("currPosTicks = %f, leftFrontPos = %d" , currPosTicks, leftFrontPos);

            if (Math.abs(currPosTicks) > Math.abs(leftFrontPos))
            {
                break;
            }
//               if (forward == 1)
//                {
//                    if (currPosTicks > leftFrontPos)
//                        break;
//                }
//                else
//                {
//                    if (currPosTicks < leftFrontPos)
//                        break;
//                }
        }


        this.driveMotors(0, 0, 0, 0);

        this.resetEncoders();
        this.resetRunMode();

    }

    /**
     * Drives the bot forward or backward in a straight line.
     * @param target distance in inches to travel.
     * @param forward indicates direction of travel.  -1 is forward 1 is backwards?
     * @param speed double value indicating the speed from 0 to 1.
     */
    public void forward(double target, int forward, double speed)
    {

        this.resetEncoders();
        resetRunMode();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(0, 500);
        pidDrive.setTolerance(1);
//       pidDrive.setContinuous();
        pidDrive.enable();

        // Set up parameters for strafe correction.
        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, 0.3);
        pidStrafe.setInputRange(-5000, 5000);
        pidStrafe.enable();

        speed *= forward;
        int leftFrontPos = this.lf.getCurrentPosition();

            if (forward == 1)
                leftFrontPos += target * ticksPerInch;
            else
                leftFrontPos -= target * ticksPerInch;

            while (mOpMode.opModeIsActive())
            {
                int frontLeftPos, frontRightPos;

                frontLeftPos = this.lf.getCurrentPosition();
                frontRightPos = this.rf.getCurrentPosition();

                double currPosTicks = (frontLeftPos + frontRightPos) / 2;


                //if the number is positive the bot is slipping right
                //if the number is negative the bot is slipping left
                //lf and rf are added because rf is reverse of lf direction.
                int wheelDifference = frontLeftPos - frontRightPos;

                // if the number is positive the bot strafed left
                // if the number is negative the bot strafed right
                int strafeDifference = this.rb.getCurrentPosition();

                // Use PID with imu input to drive in a straight line.
                // pos is right turn, neg is left turn
                double correction = pidDrive.performPID(wheelDifference);
//                mOpMode.telemetry.addData("correction ", "correction: " + correction + " wheelDif: " + wheelDifference);

                double strafeCorrection = pidStrafe.performPID(strafeDifference);
                //mOpMode.telemetry.addData("strafeCorrection ", "correction: " + strafeCorrection + " strafeDifference: " + strafeDifference);

               //Logging.log("left Encoder = %d, Right Encoder = %d wheelDifference = %d correction = %f currPosTicks = %f", this.lf.getCurrentPosition(), this.rf.getCurrentPosition(), wheelDifference, correction,currPosTicks);
               Logging.log("left Encoder = %d, Right Encoder = %d wheelDifference = %d" , frontLeftPos, frontRightPos, wheelDifference);

                //this.driveMotors(speed, 0, 0, 1); //run with no PID

                if (pidDrive.onTarget())
                {
                    correction = 0;
                }
                else
                {
                    correction = correction * (speed * 0.2);
                }
                this.driveMotors(speed, (correction * forward) + 0.01, -strafeCorrection, 1); // run with PID

//               this.driveMotors(speed, 0, 0, 1);

//                mOpMode.telemetry.addData("Encoder", "left: " + lf.getCurrentPosition() + " right: " + rf.getCurrentPosition() + " strafe: " + rb.getCurrentPosition());
//                mOpMode.telemetry.update();

//               Logging.log("currPosTicks = %f, leftFrontPos = %d" , currPosTicks, leftFrontPos);

               if (Math.abs(currPosTicks) > Math.abs(leftFrontPos))
               {
                  break;
               }
//               if (forward == 1)
//                {
//                    if (currPosTicks > leftFrontPos)
//                        break;
//                }
//                else
//                {
//                    if (currPosTicks < leftFrontPos)
//                        break;
//                }
            }


        this.driveMotors(0, 0, 0, 0);

        this.resetEncoders();
        this.resetRunMode();

    }

    /**
     * Drives the bot right or backward in a straight line.
     * @param target distance in inches to travel.
     * @param right indicates direction of travel.  -1 is right 1 is backwards?
     * @param speed double value indicating the speed from 0 to 1.
     * @param seconds Will stop running to position if time exceeds this time
     */
    public void strafe(double target, int right, double speed, ImuHardware imuControl, int seconds)
    {
        //Init the starting angle
        imuControl.resetAngle();
        double targetAngle = imuControl.getAngle();

        //reset the encoders
        this.resetEncoders();
        this.resetRunMode();

        //control the error of forward and backwards motion
        pidDrive = new PIDController(0.0001, 0, 0.000);
        //pidDrive = new PIDController(0.0005, 0, 0.000);
        //control the error of heading
        pidRotateImu = new PIDController(.04, .000, .000);

        // Set up parameters for turn correction.
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.2);
        pidDrive.setInputRange(0, 5000);
        pidDrive.enable();

        // Set up parameters for strafe correction.
//        pidStrafe.reset();
//        pidStrafe.setSetpoint(0);
//        pidStrafe.setOutputRange(0, .15);
//        pidStrafe.setInputRange(-5000, 5000);
//        pidStrafe.enable();

        pidRotateImu.reset();
        pidRotateImu.setSetpoint(targetAngle);
        pidRotateImu.setInputRange(0, 30);
        pidRotateImu.setOutputRange(0, 0.2);
        //pidRotateOd.setTolerance(.15);
        pidRotateImu.enable();

        speed *= right;
        int strafeTargetPos = this.rb.getCurrentPosition();
        strafeTargetPos += target * ticksPerInch;

        time.reset();
        time.startTime();

        while ((Math.abs(this.rb.getCurrentPosition()) <= strafeTargetPos) && mOpMode.opModeIsActive() && seconds > time.seconds())
        {
            //if the number is positive the bot is slipping forward
            //if the number is negative the bot is slipping backwards
            //lf and rf are added because rf is reverse of lf direction.
            int yDifference = ((this.lf.getCurrentPosition() + this.rf.getCurrentPosition()) / 2);

            int direction = 1;
            if (yDifference < 0)
                direction = direction * -1;

            // if the number is positive the bot strafed left
            // if the number is negative the bot strafed right
            int strafeDifference = this.rb.getCurrentPosition();

            double angle = imuControl.getAngle();

            // Use PID with imu input to drive in a straight line.
            // pos is right turn, neg is left turn
            double forwardCorrection = pidDrive.performPID(yDifference);
            mOpMode.telemetry.addData("correction ", "correction: " + forwardCorrection + " wheelDif: " + yDifference);
            Logging.log("correction: " + forwardCorrection + " yDifference: " + yDifference);

            double strafeCorrection = pidStrafe.performPID(strafeDifference);
            mOpMode.telemetry.addData("strafeCorrection ", "correction: " + strafeCorrection + " strafeDifference: " + strafeDifference);

            double headingError = pidRotateImu.performPID(angle);

            //this.driveMotors(0, (-headingError), speed, 1); // run with PID
            this.driveMotors(-forwardCorrection, (-headingError), speed, 1); // run with PID
//            Logging.log("heading: %f angle: %f headingError: %f", targetAngle,angle, headingError);
            Logging.log("forwardCorrection = %f, speed = %f ", -forwardCorrection, speed);
//            mOpMode.telemetry.addData("Encoder", "left: " + lf.getCurrentPosition() + " right: " + rf.getCurrentPosition() + " strafe: " + rb.getCurrentPosition());
//            mOpMode.telemetry.update();
        }
        this.driveMotors(0, 0, 0, 0);
    }

    /**
     * Drives the bot right or backward in a straight line.
     * @param target distance in inches to travel.
     * @param right indicates direction of travel.  1 is right -1 is left
     * @param speed double value indicating the speed from 0 to 1.
     */

    public void strafe(double target, int right, double speed, ImuHardware imuControl)
    {
        //Init the starting angle
        imuControl.resetAngle();
        double targetAngle = imuControl.getAngle();

        //reset the encoders
        this.resetEncoders();
        this.resetRunMode();

        //control the error of forward and backwards motion
        pidDrive = new PIDController(0.0001, 0, 0.000);
        //pidDrive = new PIDController(0.0005, 0, 0.000);
        //control the error of heading
        pidRotateImu = new PIDController(.04, .000, .000);

        // Set up parameters for turn correction.
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        //pidDrive.setOutputRange(0, 0.2);
        pidDrive.setOutputRange(0, 0.2);
        pidDrive.setInputRange(0, 5000);
        pidDrive.enable();

        // Set up parameters for strafe correction.
//        pidStrafe.reset();
//        pidStrafe.setSetpoint(0);
//        pidStrafe.setOutputRange(0, .15);
//        pidStrafe.setInputRange(-5000, 5000);
//        pidStrafe.enable();

        pidRotateImu.reset();
        pidRotateImu.setSetpoint(targetAngle);
        pidRotateImu.setInputRange(0, 30);
        pidRotateImu.setOutputRange(0, 0.2);
        //pidRotateOd.setTolerance(.15);
        pidRotateImu.enable();

        speed *= right;
        int strafeTargetPos = this.rb.getCurrentPosition();
        strafeTargetPos += target * ticksPerInch;

        while ((Math.abs(this.rb.getCurrentPosition()) <= strafeTargetPos) && mOpMode.opModeIsActive())
        {
            //if the number is positive the bot is slipping forward
            //if the number is negative the bot is slipping backwards
            //lf and rf are added because rf is reverse of lf direction.
            int yDifference = ((this.lf.getCurrentPosition() + this.rf.getCurrentPosition()) / 2);

            int direction = 1;
            if (yDifference < 0)
               direction = direction * -1;

            // if the number is positive the bot strafed left
            // if the number is negative the bot strafed right
            int strafeDifference = this.rb.getCurrentPosition();

            double angle = imuControl.getAngle();

            // Use PID with imu input to drive in a straight line.
            // pos is right turn, neg is left turn
            double forwardCorrection = pidDrive.performPID(yDifference);
            mOpMode.telemetry.addData("correction ", "correction: " + forwardCorrection + " wheelDif: " + yDifference);
            Logging.log("correction: " + forwardCorrection + " yDifference: " + yDifference);

            double strafeCorrection = pidStrafe.performPID(strafeDifference);
            mOpMode.telemetry.addData("strafeCorrection ", "correction: " + strafeCorrection + " strafeDifference: " + strafeDifference);

            double headingError = pidRotateImu.performPID(angle);

           //this.driveMotors(0, (-headingError), speed, 1); // run with PID
            this.driveMotors(-forwardCorrection, (-headingError), speed, 1); // run with PID
//            Logging.log("heading: %f angle: %f headingError: %f", targetAngle,angle, headingError);
            Logging.log("forwardCorrection = %f, speed = %f ", -forwardCorrection, speed);
//            mOpMode.telemetry.addData("Encoder", "left: " + lf.getCurrentPosition() + " right: " + rf.getCurrentPosition() + " strafe: " + rb.getCurrentPosition());
//            mOpMode.telemetry.update();
        }
        this.driveMotors(0, 0, 0, 0);
    }


    public void rotateOd(int degrees, double power) throws InterruptedException
    {
       Logging.log("#rotateOd degrees = %d  power = %f", degrees, power);

       int startPos = this.rb.getCurrentPosition();
       double targetPos = degrees * ticksPerDegree;
       pidRotateOd.reset();
       pidRotateOd.setSetpoint(targetPos);
       pidRotateOd.setInputRange(0, targetPos*2);
       pidRotateOd.setOutputRange(.15, power);
       pidRotateOd.setTolerance(.15);
       pidRotateOd.enable();

       // Proportional factor can be found by dividing the max desired pid output by
       // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
       // to get a P factor of .003. This works for the robot we testing this code with.
       // Your robot may vary but this way finding P works well in most situations.
       double p = Math.abs(power/targetPos);

       // Integrative factor can be approximated by diving P by 100. Then you have to tune
       // this value until the robot turns, slows down and stops accurately and also does
       // not take too long to "home" in on the setpoint. Started with 100 but robot did not
       // slow and overshot the turn. Increasing I slowed the end of the turn and completed
       // the turn in a timely manner
       double i = p / 200.0;

       //Set PID parameters based on power and ticks to travel.
//       pidRotateOd.setPID(p, i, 0);

       mOpMode.telemetry.addData("rotateOd1", "startPos:  %d   targetPos: %f ", startPos,targetPos);
       mOpMode.telemetry.update();

       Logging.log("#rotateOd startPos:  %d   targetPos: %f ", startPos,targetPos);

       int onTargetCount = 0;

       do
       {
          power = pidRotateOd.performPID(this.rb.getCurrentPosition() - startPos); // power will be + on left turn.
          this.driveMotors(0, power, 0, 1);

          //mOpMode.telemetry.addData("rotateOd2", "startPos:  %d   targetPos: %f ", startPos,targetPos);
          //mOpMode.telemetry.addData("rotateOd2", "power: %f currPos:  %d", power, this.rb.getCurrentPosition() - startPos);
          //mOpMode.telemetry.update();

          Logging.log("#rotateOd targetPos: %f power: %f currPos: %d degrees: %f",targetPos, power, this.rb.getCurrentPosition() - startPos, (this.rb.getCurrentPosition() - startPos)/ticksPerDegree);

          if (pidRotateOd.onTarget())
          {
             onTargetCount++;
             Logging.log("#onTargetCount: %d",onTargetCount);

          }

       } while (mOpMode.opModeIsActive() && (onTargetCount < 8));

       //Kill motors
       this.driveMotors(0, 0, 0, 0);

       mOpMode.telemetry.addData("rotateOd3", "startPos:  %d   targetPos: %f ", startPos,targetPos);
       mOpMode.telemetry.addData("rotateOd3", "currPos:  %d  degrees: %f", this.rb.getCurrentPosition() - startPos, (this.rb.getCurrentPosition() - startPos)/ticksPerDegree);
       mOpMode.telemetry.update();

       Logging.log("#rotateOd complete  currPos:  %d  degrees: %f",  this.rb.getCurrentPosition() - startPos, (this.rb.getCurrentPosition() - startPos)/ticksPerDegree);
    }

    public enum DirectionType
    {
       LEFT,
       RIGHT
    }


    public void rotate(double degrees, ImuHardware imuControl) throws InterruptedException, IOException
    {
       //double degrees = 30.0;
       double power = 0.0;
       int directionInt = 1;
       int counter = 0;

       if (degrees < 0)
       {
          degrees = Math.abs(degrees);
          directionInt = -1;
       }

       if (degrees == 30)
       {
          pidRotateImu = new PIDController(.04, .0001, .067);   // 30 degrees
       }
       else if (degrees == 45)
       {
          pidRotateImu = new PIDController(.04, .0001, .067);   // 45 degrees
       }
       else if (degrees == 90)
       {
          pidRotateImu = new PIDController(.04, .0001, .11);   // 90 degrees
       }
       else
       {
          pidRotateImu = new PIDController(.04, .0001, .067);   // ?? degrees
       }

       pidRotateImu.reset();
       pidRotateImu.setSetpoint(degrees);
       pidRotateImu.setInputRange(0, degrees);
       pidRotateImu.setOutputRange(0, 1);
       pidRotateImu.setTolerance(.4);
       pidRotateImu.enable();

       int onTargetCount = 0;
       int onTargetCountTotal = 0;
       // restart imu angle tracking.
       imuControl.resetAngle();

       double currAngle = 0.0;
       double remainingAngle = 0.0;

       do
       {
          currAngle = imuControl.getAngle();
          remainingAngle = degrees - currAngle;     // 90-60 = 30


          power = pidRotateImu.performPID(currAngle); // power will be + on left turn.
          this.driveMotors(0, power * directionInt, 0, 1);
          Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f", imuControl.getHeading(), power, imuControl.getAngle());

          if (pidRotateImu.onTarget())
          {
             onTargetCount++;
             onTargetCountTotal++;
             Logging.log("onTargetCount %d", onTargetCount);
          }
          else
          {
             onTargetCount = 0;
          }

          counter++;
       }
       while (mOpMode.opModeIsActive() && onTargetCount < 1 && counter < (degrees * 2));

       this.driveMotors(0, 0, 0, 1);
       Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f", imuControl.getHeading(), power, imuControl.getAngle());
       Logging.log("completed rotate of angle %f", degrees);
       mOpMode.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imuControl.getHeading());
       mOpMode.telemetry.update();
    }


   public void rotate2(double degrees, ImuHardware imuControl, int seconds) throws InterruptedException, IOException
   {
       time.reset();
       time.startTime();

      double power = 0.0;
      int directionInt = 1;
      int counter = 0;

      if (degrees < 0)
      {
         degrees = Math.abs(degrees);
         directionInt = -1;
      }

//      if (degrees == 30)
//      {
//         pidRotateImu = new PIDController(.04, .0001, .067);   // 30 degrees
//      }
//      else if (degrees == 45)
//      {
//         pidRotateImu = new PIDController(.04, .0001, .067);   // 45 degrees
//      }
//      else if (degrees == 90)
//      {
//         pidRotateImu = new PIDController(.04, .0001, .11);   // 90 degrees
//      }
//      else
      {
         pidRotateImu = new PIDController(.035, .0002, .067);   // ?? degrees
      }

      pidRotateImu.reset();
      pidRotateImu.setSetpoint(degrees);
      pidRotateImu.setInputRange(0, degrees+10);
      pidRotateImu.setOutputRange(0.15, 1);
      pidRotateImu.setTolerance(.4);
      pidRotateImu.enable();

      int onTargetCount = 0;
      int onTargetCountTotal = 0;
      // restart imu angle tracking.
      imuControl.resetAngle();

      double currAngle = 0.0;
      double remainingAngle = 0.0;

      do
      {
         currAngle = imuControl.getAngle();
         remainingAngle = degrees - currAngle;     // 90-60 = 30


         power = pidRotateImu.performPID(Math.abs(currAngle)); // power will be + on left turn.
         this.driveMotors(0, power * directionInt, 0, 1);
//         Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f", imuControl.getHeading(), power, imuControl.getAngle());

         if (pidRotateImu.onTarget())
         {
            onTargetCount++;
            onTargetCountTotal++;
//            Logging.log("onTargetCount %d", onTargetCount);
         }
         else
         {
            onTargetCount = 0;
         }

         counter++;
      }
      while (mOpMode.opModeIsActive() && onTargetCount < 3 && counter < (degrees * 2) && time.seconds() < seconds);

      this.driveMotors(0, 0, 0, 1);
      Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f counter: %d", imuControl.getHeading(), power, imuControl.getAngle(),counter);
      Logging.log("completed rotate of angle %f", degrees);
      mOpMode.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imuControl.getHeading());
      mOpMode.telemetry.update();
   }

    public void rotate2(double degrees, ImuHardware imuControl) throws InterruptedException, IOException
    {
        time.reset();
        time.startTime();

        double power = 0.0;
        int directionInt = 1;
        int counter = 0;

        if (degrees < 0)
        {
            degrees = Math.abs(degrees);
            directionInt = -1;
        }

//      if (degrees == 30)
//      {
//         pidRotateImu = new PIDController(.04, .0001, .067);   // 30 degrees
//      }
//      else if (degrees == 45)
//      {
//         pidRotateImu = new PIDController(.04, .0001, .067);   // 45 degrees
//      }
//      else if (degrees == 90)
//      {
//         pidRotateImu = new PIDController(.04, .0001, .11);   // 90 degrees
//      }
//      else
        {
            pidRotateImu = new PIDController(.035, .0002, .067);   // ?? degrees
        }

        pidRotateImu.reset();
        pidRotateImu.setSetpoint(degrees);
        pidRotateImu.setInputRange(0, degrees+10);
        pidRotateImu.setOutputRange(0.15, 1);
        pidRotateImu.setTolerance(.4);
        pidRotateImu.enable();

        int onTargetCount = 0;
        int onTargetCountTotal = 0;
        // restart imu angle tracking.
        imuControl.resetAngle();

        double currAngle = 0.0;
        double remainingAngle = 0.0;

        do
        {
            currAngle = imuControl.getAngle();
            remainingAngle = degrees - currAngle;     // 90-60 = 30


            power = pidRotateImu.performPID(Math.abs(currAngle)); // power will be + on left turn.
            this.driveMotors(0, power * directionInt, 0, 1);
//         Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f", imuControl.getHeading(), power, imuControl.getAngle());

            if (pidRotateImu.onTarget())
            {
                onTargetCount++;
                onTargetCountTotal++;
//            Logging.log("onTargetCount %d", onTargetCount);
            }
            else
            {
                onTargetCount = 0;
            }

            counter++;
        }
        while (mOpMode.opModeIsActive() && onTargetCount < 3 && counter < (degrees * 2));

        this.driveMotors(0, 0, 0, 1);
        Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f counter: %d", imuControl.getHeading(), power, imuControl.getAngle(),counter);
        Logging.log("completed rotate of angle %f", degrees);
//        mOpMode.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imuControl.getHeading());
//        mOpMode.telemetry.update();
    }


    public void rotateMez(double degrees, double power, ImuHardware imuControl) throws InterruptedException, IOException
    {
       pidRotateImu.reset();
       pidRotateImu.setSetpoint(degrees);
       pidRotateImu.setInputRange(0, degrees + degrees / 10);
       pidRotateImu.setOutputRange(.14, 1);
       pidRotateImu.setTolerance(.4);
       pidRotateImu.enable();

       int onTargetCount = 0;
       int onTargetCountTotal = 0;
       // restart imu angle tracking.
       imuControl.resetAngle();

       double currAngle = 0.0;
       double remainingAngle = 0.0;
       if (degrees < 0)
       {
          // On right turn we have to get off zero first.
          while (mOpMode.opModeIsActive() && imuControl.getAngle() == 0)
          {
             this.driveMotors(0, power, 0, 1);
             sleep(100);

          }
       }
       else
       {
          do
          {
             currAngle = imuControl.getAngle();
             remainingAngle = degrees - currAngle;

             power = -1.0;//pidRotateImu.performPID(currAngle); // power will be + on left turn.
             this.driveMotors(0, -1, 0, 1);
             Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f", imuControl.getHeading(), power, imuControl.getAngle());

          }
          while (mOpMode.opModeIsActive() && remainingAngle > 30.0);

          Logging.log("remaining angle %f", remainingAngle);
          pidRotateImu.setOutputRange(.13, 1);
          do
          {
             currAngle = imuControl.getAngle();
             remainingAngle = degrees - currAngle;     // 90-60 = 30
             pidRotateImu.setSetpoint(degrees);

             power = pidRotateImu.performPID(currAngle); // power will be + on left turn.
             this.driveMotors(0, -power, 0, 1);
             Logging.log("%.2f Deg. (Heading)  power: %f  getAngle() %f", imuControl.getHeading(), power, imuControl.getAngle());

             if (pidRotateImu.onTarget())
             {
                pidRotateImu.setOutputRange(.05, 1);
                onTargetCount++;
                onTargetCountTotal++;
                Logging.log("onTargetCount %d", onTargetCount);
             }
             else
             {
                onTargetCount = 0;
             }

          }
          while (mOpMode.opModeIsActive() && onTargetCount < 5 && onTargetCountTotal < 10);
       }
       // turn the motors off.
       this.driveMotors(0, 0, 0, 1);
       Logging.log("completed rotate of angle %f", degrees);
       mOpMode.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imuControl.getHeading());
       mOpMode.telemetry.update();



    }

}
