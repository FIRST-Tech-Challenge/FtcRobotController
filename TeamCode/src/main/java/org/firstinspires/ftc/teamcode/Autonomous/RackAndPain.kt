
package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.DESIRED_DISTANCE
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_SPEED
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_STRAFE
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_TURN
import org.firstinspires.ftc.teamcode.Variables.SPEED_GAIN
import org.firstinspires.ftc.teamcode.Variables.STRAFE_GAIN
import org.firstinspires.ftc.teamcode.Variables.TURN_GAIN
import org.firstinspires.ftc.teamcode.Variables.clawToBoard
import org.firstinspires.ftc.teamcode.Variables.desiredTag
import org.firstinspires.ftc.teamcode.Variables.drive
import org.firstinspires.ftc.teamcode.Variables.slideToBoard
import org.firstinspires.ftc.teamcode.Variables.strafe
import org.firstinspires.ftc.teamcode.Variables.t
import org.firstinspires.ftc.teamcode.Variables.targetFound
import org.firstinspires.ftc.teamcode.Variables.turn
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

@TeleOp(name = "RackAndPain", group = "AprilTag")
class RackAndPain: DriveMethods() {
    var rMotorR: DcMotor? = null;
    var rMotorL: DcMotor? = null;


    override fun runOpMode() {
        //init
//        initMotorsSecondBot()
        rMotorR = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorR");
        rMotorL = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorL");
        var lClick =0;
        var rClick = 0;
        telemetry.addData("Initiating", "Rack Motor Init");
        telemetry.update()

        waitForStart()
        
        while (opModeIsActive()) {
            lClick = (rMotorL!!.getCurrentPosition())
            rClick = (rMotorR!!.getCurrentPosition())
            telemetry.addLine("Current LPos: "+lClick+"\nCurrent RPos: "+rClick);
            telemetry.update()
            var leftRack = -1;
            var rightRack = 1;
            if (gamepad1.dpad_up){
              rMotorR!!.setPower(0.2*rightRack);
              rMotorL!!.setPower(0.2*leftRack);
            }
            else if(gamepad1.dpad_down){

              rMotorR!!.setPower(-0.2*rightRack);
              rMotorL!!.setPower(-0.2*leftRack);
            }
            else{
            
              rMotorR!!.setPower(0.0*rightRack);
              rMotorL!!.setPower(0.0*leftRack);
            }

            sleep(10)
        }
    }
}
