//package org.firstinspires.ftc.teamcode;
//
//import java.nio.file.*;
//
//import android.os.Environment;
//import com.qualcomm.robotcore.eventloop.opmode.*;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import  org.firstinspires.ftc.teamcode.FileManager;
//
//@TeleOp(name = "PositionInput")
//public class PositionInput extends OpMode {
//    // File name of the storage file,
//    // which is inside the directory specified in FileManager
//    private final Path storageFile = Paths.get("position.txt");
//
//    @Override
//    public void init() {
//        telemetry.addData("Current position: ", FileManager.readFile(storageFile));
//    }
//
//    @Override
//    public void loop() {
//        telemetry.update();
//
//        String positionString = "";
//        // Green button
//        if (gamepad1.a || gamepad2.a) {
//            positionString += TeamColor.BLUE.name() + '\n';
//            positionString += TeamSide.NEAR.name();
//
//        } else if (gamepad1.b || gamepad2.b) {
//            // Red button
//            positionString += TeamColor.RED.name() + '\n';
//            positionString += TeamSide.FAR.name();
//
//        } else if (gamepad1.x || gamepad2.x) {
//            // Blue button
//            positionString += TeamColor.BLUE.name() + '\n';
//            positionString += TeamSide.FAR.name();
//
//        } else if (gamepad1.y || gamepad2.y) {
//            // Orange button
//            positionString += TeamColor.RED.name() + '\n';
//            positionString += TeamSide.NEAR.name();
//        }
//
//        if (!positionString.equals("")) {
//            FileManager.writeToFile(storageFile, positionString);
//            telemetry.addData("Current position: ", FileManager.readFile(storageFile));
//        }
//    }
//}