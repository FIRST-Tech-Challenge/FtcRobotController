//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import java.util.ArrayList;
//
//public class SubsystemBuilder {
//    private ArrayList<HardwareDevice> devices;
//    private ArrayList<DcMotorEx> motors;
//    private ArrayList<Servo> servos;
//
//    public SubsystemBuilder addDevice(HardwareDevice device) {
//        devices.add(device);
//        return this;
//    }
//
//    public SubsystemBuilder addMotor(DcMotorEx motor) {
//        motors.add(motor);
//        return this;
//    }
//
//    public SubsystemBuilder addServo(Servo servo) {
//        servos.add(servo);
//        return this;
//    }
//
//    public SubsystemBuilder setPosition(double[] position) {
//        this.position[0] = position[0];
//        this.position[1] = position[1];
//        this.position[2] = position[2];
//        return this;
//
//    }
//
//    public Subsystem
//
//    public Subsystem build() {
//        return new Subsystem(devices, motors, servos, positions);
//    }
//}
