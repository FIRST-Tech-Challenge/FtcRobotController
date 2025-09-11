package Commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SmartShooter {
    private final DcMotor leftShooter;
    private final DcMotor rightShooter;
    public SmartShooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
    }
    public void shoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(-power);
    }
    public void stop() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
    }
}
