package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.
import com.qualcomm.robotcore.hardware.DcMotor;

//import java.util.Optional;

//import com.qualcomm.robotcore.hardware.Servo;


public class Auto extends LinearOpMode {
    private final int SERVO_MAX_POSITION = 90;
    private DcMotor frontLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor backRightWheel = null;

    //MODIFY LATER ONCE PHONES ARE CONFIGED
    private String flWheelName = "";
    private String frWheelName = "";
    private String blWheelName = "";
    private String brWheelName = "";
    private DcMotor[] motorList = {frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel};
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftWheel = hardwareMap.get(DcMotor.class, flWheelName);
        frontRightWheel = hardwareMap.get(DcMotor.class, frWheelName);
        backLeftWheel = hardwareMap.get(DcMotor.class, blWheelName);
        backRightWheel = hardwareMap.get(DcMotor.class, brWheelName);
        //tests
        configWheels();
        testRun();

    }
    private void modAllWheels(String action, Double power) {

        power = power != null ? power : 0;
        switch (action) {
            case "direction_forward":
                for (DcMotor wheel : motorList) {
                    wheel.setDirection(DcMotor.Direction.FORWARD);
                }
                break;
            case "direction_reverse":
                for (DcMotor wheel : motorList) {
                    wheel.setDirection(DcMotor.Direction.REVERSE);
                }
                break;
            case "power":
                for (DcMotor wheel : motorList) {
                    //double wheelPower = power.isPresent() ? power.get() : 0;
                    wheel.setPower(power);
                }
                break;
            default:
                break;
        }
            
                
    }
    private void configWheels() {
        modAllWheels("direction_forward", null);
    }
    private void testRun() {
        modAllWheels("power", 1.0);

    }
}
