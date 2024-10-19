package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RambotsPurpleTeleOp")
public class linearOpMode extends LinearOpMode {
  private DcMotor frontLeftMotor = null, backLeftMotor = null;
  private DcMotor frontRightMotor = null, backRightMotor = null;

  private DcMotor SlideExtension = null;
  private  DcMotor SlideAbduction = null;

//  private  DcMotor MaybeIntake = null;
  private Servo ClawIntake = null;



  @Override
  public void runOpMode(){

    //initializing hardware

    //Drive Train motor
    frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
    frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
    backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
    backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

    //DcMotors for Linear slide
    SlideExtension = hardwareMap.get(DcMotor.class, "SlideExtend");
    SlideAbduction = hardwareMap.get(DcMotor.class, "SlideAbd");


    //Takers
    ClawIntake = hardwareMap.get(Servo.class, "ClawIntake");

//    MaybeIntake = hardwareMap.get(DcMotor.class, "intake");
    //Setting the direction for the motor on where to rotate

    //Orientation for drivetrain
    frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
    backRightMotor.setDirection(DcMotor.Direction.REVERSE);

    //claw
    ClawIntake.setDirection(Servo.Direction.REVERSE);


    //linear slide
    SlideExtension.setDirection(DcMotor.Direction.FORWARD);
    SlideAbduction.setDirection(DcMotor.Direction.FORWARD);




    waitForStart();


    if (isStopRequested()) return;


    while (opModeIsActive()) {

      /*
        GamePad Map

        GamePad 1 (Driver)
          Left JoyStick = lateral, diagonal, forwards and backwards movements
          Right JoyStick = Rotation of drive train
        GamePad 2 (Operator)
          Button A = toggle position of claw to open or closed (We start closed)

       */


      double ClawIntakePostion = 1;

      double y = -gamepad1.left_stick_y;
      double x = -gamepad1.left_stick_x ;
      double turn = gamepad1.right_stick_x;


      //input: theta and power
      //theta is where we want the direction the robot to go
      //power is (-1) to 1 scale where increasing power will cause the engines to go faster
      double theta = Math.atan2(y,x);
      double power = Math.hypot(x,y);
      double sin = Math.sin(theta - Math.PI/4);
      double cos = Math.cos(theta - Math.PI/4);
      //max variable allows to use the motors at its max power with out disabling it
      double max = Math.max(Math.abs(sin),Math.abs(cos));


      double leftFront = power * cos/max + turn;
      double rightFront = power * cos/max - turn;
      double leftRear = power * sin/max + turn;
      double rightRear = power * sin/max - turn;



      //Prevents the motors exceeding max power thus motors will not seize and act sporadically
      if ((power + Math.abs(turn))>1){
        leftFront /= power  + turn;
        rightFront /= power  - turn;
        leftRear /= power  + turn;
        rightRear /= power  - turn;
      }

      //if A on the controller is pressed it will check if the claw is closed
      if(gamepad1.a){
        //if it's closed
        if(ClawIntakePostion == 1){
            //Set claw position to open
            ClawIntakePostion = 0;
        }
        //if it's open
        else{
          //Set claw position to closed
          ClawIntakePostion = 1;
        }
      }


      //Power to the wheels
      frontLeftMotor.setPower(leftFront);
      backLeftMotor.setPower(leftRear);
      frontRightMotor.setPower(rightFront);
      backRightMotor.setPower(rightRear);


      //Power to the Claw
      ClawIntake.setPosition(ClawIntakePostion);

     
    }
  }
}
