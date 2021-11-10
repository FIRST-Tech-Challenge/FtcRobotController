package org.firstinspires.ftc.teamcode.AdrianControls;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public abstract class AdrianMecanumControls extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotorEx rightEncoder, leftEncoder, sideEncoder;
    protected VuforiaLocalizer vuforia;
    protected static final String VUFORIA_KEY = "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";



    protected void initializeHardware() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        sideEncoder = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    protected int GetRightEncoderValue() {
        return rightEncoder.getCurrentPosition();

    }

    protected int GetLeftEncoderValue() {
        return leftEncoder.getCurrentPosition();

    }

    protected int GetSideEncoderValue() {
        return sideEncoder.getCurrentPosition();

    }

    protected void GridMecanumMovement(double xvalue, double yvalue, double speed    )  {

        double yEncoderCount = yvalue*22000;
        double xEncoderCount = xvalue*23000;

if (opModeIsActive()) {
            while (rightEncoder.getCurrentPosition() < yEncoderCount) {
                leftRear.setPower(speed);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                rightRear.setPower(speed);

            }

            leftRear.setPower(0.0);
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            rightRear.setPower(0.0);


            while (sideEncoder.getCurrentPosition() < xEncoderCount) {
                if (xvalue > 0) {
                    leftRear.setPower(speed);
                    leftFront.setPower(-1 * speed);
                    rightFront.setPower(speed);
                    rightRear.setPower(-1 * speed);

                }
                else {
                    leftRear.setPower(-1 * speed);
                    leftFront.setPower(speed);
                    rightFront.setPower(-1 * speed);
                    rightRear.setPower(speed);

                }


            }
            leftRear.setPower(0.0);
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            rightRear.setPower(0.0);
        }



    }





    }




