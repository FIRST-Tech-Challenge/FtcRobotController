//package org.firstinspires.ftc.teamcode.subsystems;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.Twist2dDual;
//
//import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
//import org.firstinspires.ftc.teamcode.roadrunner.tuning.GoBildaPinpointDriver;
//
//public class mainLocalizer implements Localizer {
//    GoBildaPinpointDriver odo;
//    double oldTime = 0;
//    public mainLocalizer()
//    {
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(-84.0, -168.0, headingOffsetDeg);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.recalibrateIMU();
//        odo.resetPosAndIMU();
//
////        telemetry.addData("Status", "Initialized");
////        telemetry.addData("X offset", odo.getXOffset());
////        telemetry.addData("Y offset", odo.getYOffset());
////        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
////        telemetry.addData("Device Scalar", odo.getYawScalar());
////        telemetry.update();
//
//    }
//    @Override
//    public Twist2dDual<Time> update() {
//
//        return null;
//    }
//
//
//
//
//}
