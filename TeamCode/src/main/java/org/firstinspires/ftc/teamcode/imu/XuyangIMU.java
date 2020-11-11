//package org.firstinspires.ftc.teamcode.imu;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.robotcore.external.navigation.*;
//import org.firstinspires.ftc.teamcode.utility.PoseOrientation;
//
//public class XuyangIMU {
//
//        BNO055IMU imu;
//        OpMode opmode;
//
//        Orientation lastAngle = new Orientation();
//        Position lastPos = new Position();
//
//        double deltaAngle;
//        Position deltaPos;
//
//        volatile boolean activated = false;
//
//        public XuyangIMU(OpMode opMode) {
//
//            this.opmode = opMode;
//
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//            parameters.mode = BNO055IMU.SensorMode.IMU;
//            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parameters.loggingEnabled = false;
//
//            //TODO assumption that that thing is named "imu"
//            imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");
//
//            imu.initialize(parameters);
//
//            //watchdog
//            long start = System.currentTimeMillis();
//            //make sure the imu gyro is calibrated before continuing.
//            while (!imu.isGyroCalibrated()) {
//                if (System.currentTimeMillis() - start > 500) {
//                    opmode.telemetry.addData("IMU", "watchdig quit");
//                    opmode.telemetry.update();
//
//                    break;
//                }
//
//                //wait
//                opmode.telemetry.addData("IMU", ("Loading" + Math.random()));
//                opmode.telemetry.update();
//            }
//
//            opmode.telemetry.addData("IMU", "startup done");
//            opmode.telemetry.update();
//        }
//
//        public void start() {
//            resetAngle();
//        }
//
//        /**
//         * Resets the cumulative angle tracking to zero.
//         */
//        private void resetAngle() {
//            lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            lastPos = imu.getPosition();
//
//            deltaAngle = 0;
//            deltaPos = new Position();
//        }
//
//        public void stop() {
//        }
//
//        /**
//         * call this as fast as you can!
//         * return difference in orientation to  when you last called it
//         * (x,y) is currently broken. But it isn't accurate so you should never use it
//         *
//         * @return defaults to mm for position x and y
//         */
//        public PoseOrientation getDeltaPosition() {
//            updatePos();
//            return new PoseOrientation(deltaPos.x, deltaPos.y, deltaAngle);
//        }
//
//        /**
//         * Based on the last time we called get angle, take the smallest possible rotation difference
//         * trig angles
//         */
//        private void updatePos() {
//            //z axis is the one we want
//
//            Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            Position pos = imu.getPosition();
//
//            //=========ANGLE=============
//            double deltaAngle = angle.firstAngle - lastAngle.firstAngle;
//
//            if (deltaAngle < -180)
//                deltaAngle += 360;
//
//            else if (deltaAngle > 180)
//                deltaAngle -= 360;
//
//            this.deltaAngle = deltaAngle;
//
//            lastAngle = angle;
//            //========POSITION==========
//            opmode.telemetry.addData("imu raw read", pos.x);
//
//            deltaPos.x = pos.x - lastPos.x;
//            deltaPos.y = pos.y - lastPos.y;
//
//            lastPos = pos;
//        }
//
//        public void printposition(PoseOrientation toprint) {
//            if (toprint != null) {
//                opmode.telemetry.addData("IMU-Position (mm) (rot)", toprint.x + " " + toprint.y + " " + toprint.rot);
//            } else {
//                opmode.telemetry.addData("IMU", "offline");
//            }
//        }
//    }
//}
