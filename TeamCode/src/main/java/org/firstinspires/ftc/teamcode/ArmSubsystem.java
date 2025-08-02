//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import java.util.ArrayList;
//import java.util.LinkedList;                                                                                                                                                                                                                                                                                                                                                                                                                                                                               -
//
//public class ArmSubsystem extends Subsystem {
//    private PDFLController Controller1;
//
//
//    public ArmSubsystem(ArrayList<PDFLController> PDFLController, ArrayList<HardwareDevice> devices, ArrayList<DcMotorEx> motors, ArrayList<Servo> servos, double[] position) {
//      super(devices, motors, servos, position);
//      this.PDFLController = PDFLController;
//
//    }
//
//
//    public void setPosition(int pos) {
//        Controller1.setTarget(pos);
//    }
//    public void armUpdate(int pos) {
//        Controller1.update(pos);
//    }
//}
