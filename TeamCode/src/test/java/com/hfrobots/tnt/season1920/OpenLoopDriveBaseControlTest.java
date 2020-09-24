/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.season1920;

import com.hfrobots.tnt.fakes.control.FakeOnOffButton;
import com.hfrobots.tnt.fakes.control.FakeRangeInput;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.fakes.drive.FakeExtendedDcMotor;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.fakes.sensors.FakeBNO055IMU;
import com.hfrobots.tnt.fakes.FakeHardwareMap;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class OpenLoopDriveBaseControlTest {
    private FakeHardwareMap hardwareMap;

    private FakeExtendedDcMotor leftFrontDriveMotor;

    private FakeExtendedDcMotor leftRearDriveMotor;

    private FakeExtendedDcMotor rightFrontDriveMotor;

    private FakeExtendedDcMotor rightRearDriveMotor;

    private FakeBNO055IMU imu;

    private RoadRunnerMecanumDriveREV drivebase;

    protected FakeRangeInput leftStickY;

    protected FakeRangeInput leftStickX;

    protected FakeRangeInput rightStickX;

    protected RangeInput driveInvertedButton;

    protected RangeInput driveFastButton;

    protected OnOffButton driveBumpStrafeRightButton;

    protected OnOffButton driveBumpStrafeLeftButton;

    protected DriverControls controls;

    @Before
    public void setUp() {
        hardwareMap = new FakeHardwareMap();

        imu = new FakeBNO055IMU();
        hardwareMap.addDevice("imu", imu);

        leftFrontDriveMotor = new FakeExtendedDcMotor();
        hardwareMap.addDevice("leftFrontDriveMotor", leftFrontDriveMotor);

        leftRearDriveMotor = new FakeExtendedDcMotor();
        hardwareMap.addDevice("leftRearDriveMotor", leftRearDriveMotor);

        rightFrontDriveMotor = new FakeExtendedDcMotor();
        hardwareMap.addDevice("rightFrontDriveMotor", rightFrontDriveMotor);

        rightRearDriveMotor = new FakeExtendedDcMotor();
        hardwareMap.addDevice("rightRearDriveMotor", rightRearDriveMotor);

        RoadRunnerMecanumDriveREV drivebase = new RoadRunnerMecanumDriveREV(new SkystoneDriveConstants(), hardwareMap, false);

        OpenLoopMecanumKinematics kinematics = new OpenLoopMecanumKinematics(drivebase);

        // FIXME: Create the driver controls

        // First, you need to create fake versions of all of the buttons above
        leftStickY = new FakeRangeInput();

        leftStickX = new FakeRangeInput();

        rightStickX = new FakeRangeInput();

        driveInvertedButton = new FakeRangeInput();

        driveFastButton = new FakeRangeInput();

        driveBumpStrafeRightButton = new FakeOnOffButton();

        driveBumpStrafeLeftButton = new FakeOnOffButton();

        controls = DriverControls.builder()
                .kinematics(kinematics)
                .leftStickY(leftStickY)
                .leftStickX(leftStickX)
                .rightStickX(rightStickX)
                .rightTrigger(driveInvertedButton)
                .leftTrigger(driveFastButton)
                .rightBumper(driveBumpStrafeRightButton)
                .leftBumper(driveBumpStrafeLeftButton).build();
    }

    @Test
    public void openLoopControl() {
       // drive straight
        leftStickY.setCurrentPosition(-1);
        controls.periodicTask();

        double leftFrontPower = leftFrontDriveMotor.getPower();
        double leftRearPower = leftRearDriveMotor.getPower();
        double rightFrontPower = rightFrontDriveMotor.getPower();
        double rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertEquals(leftFrontPower, leftRearPower, .01);
        Assert.assertEquals(rightFrontPower, rightRearPower, .01);


        // stop all movement or standstill
        leftStickY.setCurrentPosition(0);
        leftStickX.setCurrentPosition(0);
        rightStickX.setCurrentPosition(0);
        controls.periodicTask();

        leftFrontPower = leftFrontDriveMotor.getPower();
        leftRearPower = leftRearDriveMotor.getPower();
        rightFrontPower = rightFrontDriveMotor.getPower();
        rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertEquals(0, leftFrontPower, .02);
        Assert.assertEquals(0, leftRearPower, .02);
        Assert.assertEquals(0, rightFrontPower, .02);
        Assert.assertEquals(0, rightRearPower, .02);

        leftStickY.setCurrentPosition(0);
        leftStickX.setCurrentPosition(0);
        rightStickX.setCurrentPosition(-1);
        controls.periodicTask();

        leftFrontPower = leftFrontDriveMotor.getPower();
        leftRearPower = leftRearDriveMotor.getPower();
        rightFrontPower = rightFrontDriveMotor.getPower();
        rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertTrue(leftFrontPower < 0);
        Assert.assertTrue(leftRearPower < 0);
        Assert.assertTrue(rightFrontPower > 0);
        Assert.assertTrue(rightRearPower > 0);

        leftStickY.setCurrentPosition(0);
        leftStickX.setCurrentPosition(-1);
        rightStickX.setCurrentPosition(0);
        controls.periodicTask();

        leftFrontPower = leftFrontDriveMotor.getPower();
        leftRearPower = leftRearDriveMotor.getPower();
        rightFrontPower = rightFrontDriveMotor.getPower();
        rightRearPower = rightRearDriveMotor.getPower();

        Assert.assertTrue(leftFrontPower < 0);
        Assert.assertTrue(leftRearPower > 0);
        Assert.assertTrue(rightFrontPower > 0);
        Assert.assertTrue(rightRearPower < 0);

    }
}
