import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomous test", group="autonomous")

public class BlueCorner extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double moveSpeed = 0.5;


    private void DriveInDirection(String direction, double power, double time){
        time*=1000;
        if(direction.equals("FORWARDS")){
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("BACKWARDS")){
            rightFrontDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("LEFT")){
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("RIGHT")){
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        DriveInDirection("FORWARDS", moveSpeed, 1.75);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

