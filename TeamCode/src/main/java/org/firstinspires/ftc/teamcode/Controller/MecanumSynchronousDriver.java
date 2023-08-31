package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Logging;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.io.IOException;

/**
 * MecanumSynchronousDriver serves as an interface to the drive motors giving the user directional
 * methods to command the bot.  The movements are backed up by PID controller to ensure the bot
 * stays on cource.  This class is intended to be used by autonomous programs.
 */
public class MecanumSynchronousDriver extends MechanicalDriveBase
{

   /**
    * Number of ticks covered in one inch of movement for the odometery wheel.
    * NOTE: value may need fine tuning.
    */
   private final double ticksPerInch = (8192 * 1) / (2 * 3.1415); // == 1303

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



    /**
     * Constructor for MechanicalDriveBase from the hardware map
     *
     * @param hardwareMap the hardware map
     */
    public MecanumSynchronousDriver(HardwareMap hardwareMap, LinearOpMode opMode) throws IOException
    {
       super(opMode.hardwareMap);

       //logger.setup();
       //logger.log("Starting MecanumSynchronousDriver");

       mOpMode = opMode;
       opMode.telemetry.update();

       //The input can be a large value, therefore the values of Kp needs to be small to compensate.
       //
       // Set PID proportional value to produce non-zero correction value when robot veers off
       // straight line. P value controls how sensitive the correction is.
       pidDrive = new PIDController(0.001, 0, 0);

       pidStrafe = new PIDController(0.001, 0, 0);

        // .05
    }

   /**
    * Drives the bot forward or backward in a straight line.
    * @param target distance in inches to travel.
    * @param forward indicates directipn of travel.  -1 is forward 1 is backwards?
    * @param speed double value indicating the speed from 0 to 1.
    */
    public void forward(double target, int forward, double speed)
    {

       // Set up parameters for turn correction.
       pidDrive.setSetpoint(0);
       pidDrive.setOutputRange(0, .19);
       pidDrive.setInputRange(-5000, 5000);
       pidDrive.enable();

       // Set up parameters for strafe correction.
       pidStrafe.setSetpoint(0);
       pidStrafe.setOutputRange(0, .15);
       pidStrafe.setInputRange(-5000, 5000);
       pidStrafe.enable();

        speed *= forward;
        int leftFrontPos = this.lf.getCurrentPosition();
        if (forward == 1)
        {
            leftFrontPos += target * ticksPerInch;
            while ((this.lf.getCurrentPosition() <= leftFrontPos) &&mOpMode.opModeIsActive())
            {
               //if the number is positive the bot is slipping right
               //if the number is negative the bot is slipping left
               //lf and rf are added because rf is reverse of lf direction.
               int wheelDifference = this.lf.getCurrentPosition() + this.rf.getCurrentPosition();

               // if the number is positive the bot strafed left
               // if the number is negative the bot strafed right
               int strafeDifference = this.rb.getCurrentPosition();

               // Use PID with imu input to drive in a straight line.
               // pos is right turn, neg is left turn
               double correction = pidDrive.performPID(wheelDifference);
               mOpMode.telemetry.addData("correction ", "correction: " + correction + " wheelDif: " + wheelDifference);

               double strafeCorrection = pidStrafe.performPID(strafeDifference);
               mOpMode.telemetry.addData("strafeCorrection ", "correction: " + strafeCorrection + " strafeDifference: " + strafeDifference);


               //this.driveMotors(speed, 0, 0, 1); //run with no PID

               this.driveMotors(speed, correction, -strafeCorrection, 1); // run with PID

       //         logger.log("left Encoder = %d, Right Encoder = %d ", this.lf.getCurrentPosition(), this.rf.getCurrentPosition());
               mOpMode.telemetry.addData("Encoder", "left: " + lf.getCurrentPosition() + " right: " + rf.getCurrentPosition() + " strafe: " + rb.getCurrentPosition());
               mOpMode.telemetry.update();
            }
        }
        else  //TODO: would like to not have a else here and have to repeat the above code or have to call a subjuction..
        {
            leftFrontPos -= target;
            while (this.lf.getCurrentPosition() >= leftFrontPos)
            {
                this.driveMotors(speed, 0, 0, 1);

                //logger.log("left Encoder = %d, Right Encoder = %d ", this.lf.getCurrentPosition(), this.rf.getCurrentPosition());
                //mOpMode.telemetry.addData("NOOOOOOO", this.lf.getCurrentPosition());
                //mOpMode.telemetry.update();
            }
        }
        this.driveMotors(0, 0, 0, 0);
//        encoderLogging();
    }
}
