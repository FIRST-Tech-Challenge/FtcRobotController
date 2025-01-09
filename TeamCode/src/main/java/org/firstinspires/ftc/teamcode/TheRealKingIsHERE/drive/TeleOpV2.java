package org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.DeliveryAxonNate;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.DeliveryGrippersNate;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.DeliverySlidesNate;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.DifferentialNate;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.GrippersNate;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.IntakeSlidesNate;
import org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors.WristAxonNate;

@TeleOp
public class TeleOpV2 extends OpMode {
    //MOTORS====================================================================================================================
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    //ENUMS=====================================================================================================================
    public enum SystemStates {
        START,
        HOVER_OVER_SAMPLE,
        DOWN_ON_SAMPLE,
        ARM_UP,
        READY_FOR_DELIVERY,
        DELIVER_LOW_BASKET,
        GRAB_OFF_WALL,
        DELIVER_HIGH_BAR,
        DELIVER_UP,
        DROP,
        ERROR
    }

    public SystemStates state;
    //HARDWARE==================================================================================================================
    public DeliveryGrippersNate deliveryGrippers = new DeliveryGrippersNate();
    public DifferentialNate differentialV2 = new DifferentialNate();
    public IntakeSlidesNate intakeSlides = new IntakeSlidesNate();
    public GrippersNate grippers = new GrippersNate();
    public DeliveryAxonNate deliveryAxon = new DeliveryAxonNate();
    public WristAxonNate wrist = new WristAxonNate();
    public DeliverySlidesNate deliverySlides = new DeliverySlidesNate();
    //TIMERS====================================================================================================================
    public ElapsedTime buttonTimer = new ElapsedTime();
    public ElapsedTime Timer = new ElapsedTime();


    @Override
    public void init() {
        //Motors =====================================================================================================
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        state = SystemStates.START;
        differentialV2.init(hardwareMap, telemetry);
        intakeSlides.init(hardwareMap);
        grippers.init(hardwareMap, telemetry);
        deliveryAxon.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        wrist.init(hardwareMap);


        deliverySlides.init(hardwareMap);
        //===========================================================================================================
    }

    @Override
    public void start(){
        buttonTimer.reset();
        Timer.reset();

    }

    @Override
    public void loop() {
        //MECANUM_DRIVE====================================================================================================
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.right_stick_x; //This is reversed for our turning
        drive(y, x, rx);
        //SYSTEM_STATES====================================================================================================
        switch (state) {
            case START:

                state = SystemStates.GRAB_OFF_WALL;
                break;
            case GRAB_OFF_WALL:
                break;
            case HOVER_OVER_SAMPLE:
                break;
            case DOWN_ON_SAMPLE:
                break;
            case ARM_UP:
                //TODO: Sam said to make button to release grippers, but this current program can't do that
                break;
            case READY_FOR_DELIVERY:
                break;
            case DELIVER_UP:
                break;
            case DELIVER_LOW_BASKET:
                break;
            case DROP:
                break;
        }
        telemetry.addLine("STATE: " + state.name());
        telemetry.addLine("\t Intake grippers position: " + grippers.getPosition());
        telemetry.addLine("\t Intake slides power: " + intakeSlides.getPower());
        telemetry.addLine("\t Intake slides position: " + intakeSlides.getCurrentPosition());


    }

    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
