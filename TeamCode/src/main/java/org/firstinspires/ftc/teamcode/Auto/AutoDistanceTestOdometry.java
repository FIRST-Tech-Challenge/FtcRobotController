package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDistanceTestOdometry", group="Robot")

public class AutoDistanceTestOdometry extends LinearOpMode {
    // setting up the motors using our standard naming convention
    private DcMotor BRight;
    private DcMotor BLeft;
    private DcMotor FRight;
    private DcMotor FLeft;




    // creating runtime to allow to use for time elapsed
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
// Setting up the motors and reversing the one motor to avoid having to figure out the direction
        FRight = hardwareMap.dcMotor.get("FRight");
        FLeft  = hardwareMap.dcMotor.get("FLeft");
        BRight = hardwareMap.dcMotor.get("BRight");
        BLeft  = hardwareMap.dcMotor.get("BLeft");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotor.Direction.FORWARD);
        BRight.setDirection(DcMotor.Direction.FORWARD);


        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       BLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       BRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

// setting up the value for encoder
        int encoder = 0;
        waitForStart();
        telemetry.addData("WaitForStart", "Initialized");
        telemetry.update();


//  resetting the timer once the game starts
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double Power = .2;
            telemetry.addData("Power", encoder);
            telemetry.update();
            while (encoder > -2000){
                FRight.setPower(Power);
                FLeft.setPower(Power);
                BRight.setPower(Power);
                BLeft.setPower(Power);
                encoder = FLeft.getCurrentPosition();
                telemetry.addData("Ticks:", FLeft.getCurrentPosition());
                // Show the elapsed game time and wheel power.
                telemetry.addData("Power", "%4.2f", Power);
                telemetry.update();
            }
            Power = 0;
            FRight.setPower(Power);
            FLeft.setPower(Power);
            BRight.setPower(Power);
            BLeft.setPower(Power);
        }
    }
}
