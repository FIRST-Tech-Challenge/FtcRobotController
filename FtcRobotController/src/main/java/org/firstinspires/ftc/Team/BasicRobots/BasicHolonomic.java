package Team7159.BasicRobots;

import com.arcrobotics.ftclib.hardware.motors.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.motors.*;

public class BasicHolonomic {

    public DcMotor RFMotor;
    public DcMotor RBMotor;
    public DcMotor LFMotor;
    public DcMotor LBMotor;


    public void init(HardwareMap Map){

        //  LFMotor = new Motor(Map, "FLDrive");
        //  LBMotor = new Motor(Map, "BLDrive");
        // RFMotor = new Motor(Map, "FRDrive");
        //   RBMotor = new Motor(Map, "BRDrive");
        LFMotor = Map.dcMotor.get("FLDrive");
        LBMotor = Map.dcMotor.get("BLDrive");
        RFMotor = Map.dcMotor.get("FRDrive");
        RBMotor = Map.dcMotor.get("BRDrive");

        //RFMotor.set(0.0);
        //RBMotor.set(0.0);
        //LFMotor.set(0.0);
        //LBMotor.set(0.0);
        LFMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        RFMotor.setPower(0.0);
        RBMotor.setPower(0.0);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}