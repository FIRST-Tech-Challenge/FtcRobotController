package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IMU.IMUControl;

public class MechanicalDriveBase
{
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor rf;
  
    public double leftPowerFront  = 0;
    public double rightPowerFront = 0;
    public double rightPowerBack  = 0;
    public double leftPowerBack   = 0;
    public double speed = 0;

    final double  COUNTS_PER_INCH = (8192 * 1) / (2 * 3.1415); // 1,303.835747254496
    private double heading = 0;
    IMUControl imuControl;

    /**
     * Constructor for MechanicalDriveBase from the hardware map
     *
     * @param hardwareMap the hardware map
     */
    public MechanicalDriveBase(HardwareMap hardwareMap)
    {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // Run Without Encoders
        resetRunMode();
      
        // reset encoders
        resetRunMode();

        // Brake when power set to Zero
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * We tend to set all the motor modes at once, so break it out using "extract Method" under the
     * refactor menu
     */
    protected void resetRunMode()
    {
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void resetEncoders()
    {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //make the robot stop
    public void brake()
    {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /**
     * Standard controls from a gamepad
     *
     * @param gamepad - the gamepad you want to control the drive base
     */
    public void gamepadController(Gamepad gamepad)
    {
          double drive = -gamepad.left_stick_y;
          double turn = gamepad.right_stick_x;
          double strafe = gamepad.left_stick_x;
          speed = 1 - (0.6 * gamepad.right_trigger);
          driveMotors(drive, turn, strafe, speed);
    }

    /**
     * Drive the motors according to drive, turn, strafe inputs.
     *
     * @param drive forward / backward (-1 to 1)
     * @param turn how much to turn left or right (heading) (-1 to 1)
     * @param strafe strafe (left or right = -1 to 1)
     * @param speed scale factor that is applied to all motor powers (0 to 1)
     */
      public void driveMotors(double drive, double turn, double strafe, double speed)
      {
          leftPowerFront  = (drive + turn + strafe);
          rightPowerFront = (drive - turn - strafe);
          leftPowerBack   = (drive + turn - strafe);
          rightPowerBack  = (drive - turn + strafe);

          // This code is awful.
          double maxAbsVal = maxAbsVal(leftPowerFront, leftPowerBack,
                                       rightPowerFront, rightPowerBack);
        
          maxAbsVal = Math.max(1.0, maxAbsVal);

          lf.setPower(leftPowerFront/maxAbsVal * speed);
          rf.setPower(rightPowerFront/maxAbsVal * speed);
          lb.setPower(leftPowerBack/maxAbsVal * speed);
          rb.setPower(rightPowerBack/maxAbsVal * speed);

      }

    /**
     * Returns the absolute maximum power on any drive motor.
     *
     * @return max abs power [0,1]
     */
    public double maxMotorPower()
    {
          return maxAbsVal(lf.getPower(), rf.getPower(), lb.getPower(), rb.getPower());
    }

    /**
     * maxAbsVal returns the maximum absolute value among an arbitrary number of arguments.
     *
     * @param values an arbitrary number of values.
     * @return the maximum absolute value among the numbers.
     */
      public static double maxAbsVal(double ... values){
          double mav = Double.NEGATIVE_INFINITY;
          for (double val: values) {
              mav = Math.max(mav, Math.abs(val));
          }
          return mav;
      }

    /**
     * report drivebase telemetry
     *
     * @param telemetry the telemetry object we're reporting to.
     */
    public void driveBaseTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Motors", "lf: %.2f rf: %.2f lb: %.2f rb: %.2f", leftPowerFront, rightPowerFront, leftPowerBack, rightPowerBack);
        telemetry.addData("Speed control", speed);
    }
}
