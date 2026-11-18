package org.firstinspires.ftc.teamcode.hardware;




import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class TestBench {



    private DcMotorEx motor0;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;

    private DcMotorEx transfer;
    private DcMotorEx intake;
    private DcMotorEx launcherPollen;
    private DcMotorEx launcherNectar;
    public double ticksPerRev;
    private CRServo servo;

//    RobotHardwareMap robotHardwareMap;

    public TestBench(RobotHardwareMap robotHardwareMap) {
//        this.robotHardwareMap = robotHardwareMap;
    }
    public TestBench() {
//        this.robotHardwareMap = robotHardwareMap;
    }


    public void init(HardwareMap haMap){
        motor0 = haMap.tryGet(DcMotorEx.class, "motor0");
        motor1 = haMap.tryGet(DcMotorEx.class, "motor1");
        motor2 = haMap.tryGet(DcMotorEx.class, "motor2");
        motor3 = haMap.tryGet(DcMotorEx.class, "motor3");
        servo = haMap.tryGet(CRServo.class,"pollen_servo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = setMotorBrake( haMap.get(DcMotorEx.class, "transfer_motor"),true);
//        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = setMotorBrake( haMap.get(DcMotorEx.class, "intake_motor"),true);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherPollen = setMotorBrake(haMap.get(DcMotorEx.class, "launcher_pollen"),true);
//        launcherPollen.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherNectar = setMotorBrake(haMap.get(DcMotorEx.class, "launcher_nectar"),true);
//        launcherPollen.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor3 = setMotorBrake( motor3 , true);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ticksPerRev = motor3.getMotorType().getTicksPerRev();


    }
    public DcMotorEx setMotorBrake(DcMotorEx currMotor, boolean brake){
        currMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (brake) {
            currMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            currMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        return currMotor;
    }

    public void setMotorSpeed(double speed){

//        motor3.setVelocity(speed);
        launcherPollen.setPower(speed);
        //motor3.setVelocity(1500, AngleUnit.DEGREES);
        //motor3.setVelocity(speed);
    }

    public double getMotorRev(){
        return launcherPollen.getCurrentPosition() / ticksPerRev;  //normalizing ticks to revolutions
    }
    public DcMotorEx getMotor0() {
        return launcherPollen;
    }
    public DcMotorEx getMotor1() {
        return launcherPollen;
    }

    public DcMotorEx getMotor2() {
        return motor2;
    }

    public DcMotorEx getMotor3() {
        return motor3;
    }

    public void setMotor0(DcMotorEx motor) {
        this.motor0 = motor;
    }
    public void setMotor1(DcMotorEx motor) {
        this.motor1 = motor;
    }
    public void setMotor2(DcMotorEx motor) {
        this.motor2 = motor;
    }
    public void setMotor3(DcMotorEx motor) {
        this.motor3 = motor;
    }

    public DcMotorEx getTransfer() {
        return transfer;
    }

    public void setTransfer(DcMotorEx transfer) {
        this.transfer = transfer;
    }

    public DcMotorEx getIntake() {
        return intake;
    }

    public void setIntake(DcMotorEx intake) {
        this.intake = intake;
    }

    public DcMotorEx getLauncherPollen() {
        return launcherPollen;
    }

    public void setLauncherPollen(DcMotorEx launcherPollen) {
        this.launcherPollen = launcherPollen;
    }

    public DcMotorEx getLauncherNectar() {
        return launcherNectar;
    }

    public void setLauncherNectar(DcMotorEx launcherNectar) {
        this.launcherNectar = launcherNectar;
    }

    public CRServo getServo() {
        return servo;
    }

    public void setServo(CRServo servo) {
        this.servo = servo;
    }
}
