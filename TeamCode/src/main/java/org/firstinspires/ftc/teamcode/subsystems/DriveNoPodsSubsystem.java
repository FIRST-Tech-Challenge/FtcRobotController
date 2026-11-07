//package org.firstinspires.ftc.teamcode.subsystems;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.archive.GoBildaPinpointDriver;
//
//public class DriveNoPodsSubsystem {
//
//
//    private Telemetry telemetry;
//
//    public DriveNoPodsSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
//        this.telemetry = telemetry;
//        frontRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
//        frontLeftDrive = hardwareMap.get(DcMotor.class,"FRDrive");
//        backLeftDrive = hardwareMap.get(DcMotor.class,"FRDrive");
//        backRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
//
//        // frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
//        pinpoint.setOffsets(0,0);
//
//        pinpoint.resetPosAndIMU();
//
//    }
//
//    public void driveMath(){
//
//
//    }
//
//
//}
