import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "demo rev")
public class Demo extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    DcMotor leftMotor, rightMotor;
    Servo plane;
    public static double raisePlane=0.7  ;
    public static double lowerPlane =0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        leftMotor= hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor=hardwareMap.get(DcMotor.class, "rightMotor");
        plane = hardwareMap.get(Servo.class, "plane");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        plane.setPosition(lowerPlane);

        waitForStart();

        while (opModeIsActive()){
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(gamepad1.right_stick_y);

            if (gamepad1.a){
                plane.setPosition(raisePlane);
            }
            if (gamepad1.b){
                plane.setPosition(lowerPlane);
            }
        }
    }
}
