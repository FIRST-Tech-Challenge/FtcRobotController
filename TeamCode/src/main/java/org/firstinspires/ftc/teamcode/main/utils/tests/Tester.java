package org.firstinspires.ftc.teamcode.main.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.File;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.zip.DataFormatException;

/**
 * A Tester finds all Tests and tests them unless they have an @Skippable annotation. If a Test throws an Exception or Test.then() returns false, the Test fails. If Test.then() return true, the Test passes. Results are logged via the console and the Telemetry screen on a Driver Station. Run the AutomatedTester Autonomous OpMode to create and run a Tester.
 */

// TODO: virtualize robot - in the sense of being able to run an opmode in an android emulator

public class Tester {

    private final TestConsoleLogger LOGGER;
    private final TestTelemetryLogger TELEMETRY;
    private final LinearOpMode OP_MODE;

    /**
     * Creates a new Tester, which will run all Tests it can find automatically.
     * @param opMode The LinearOpMode to receive the Telemetry object to log to from. Telemetry is not the only place things will log to, as more detailed logs will be found in the console
     */
    public Tester(LinearOpMode opMode) {
        LOGGER = new TestConsoleLogger();
        TELEMETRY = new TestTelemetryLogger(opMode.telemetry);
        OP_MODE = opMode;
        TestResources.opMode = OP_MODE;
        OP_MODE.telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        LOGGER.logStart();
        TELEMETRY.logStart();
        TELEMETRY.logDisclaimer();
        OP_MODE.sleep(3000);
        LOGGER.logSearch();
        TELEMETRY.logSearch();
        ArrayList<ArrayList<Class<? extends Test>>> tests = findMeaningfulTests();
        ArrayList<HashMap<String, Boolean>> results = new ArrayList<>();
        try {
            results = runTests(tests);
        } catch(DataFormatException e) {
            LOGGER.logFindErr();
            TELEMETRY.logFindErr();
        }
        OP_MODE.sleep(2000);
        LOGGER.logResults(results);
        LOGGER.logEnd();
        TELEMETRY.clear();
        TELEMETRY.logResults(results);
        TELEMETRY.logEnd();
        OP_MODE.sleep(10000);
    }

    public enum FailureReason {
        EXCEPTION,
        FALSEHOOD,
        DID_NOT_FAIL
    }

    private ArrayList<HashMap<String, Boolean>> runTests(ArrayList<ArrayList<Class<? extends Test>>> tests) throws DataFormatException {
        ArrayList<HashMap<String, Boolean>> results = new ArrayList<>();
        if(tests.size() != 2) {
            // fail if test arraylist is formatted incorrectly
            throw new DataFormatException("Test Arraylist must only have two elements.");
        }else{
            LOGGER.logFind();
            TELEMETRY.logFind();
            OP_MODE.sleep(1000);
            // get unit/integration tests from where they're expected to be and run each test
            ArrayList<Class<? extends Test>> unitTests = tests.get(0);
            ArrayList<Class<? extends Test>> integrationTests = tests.get(1);
            LOGGER.logUnitRun();
            TELEMETRY.clear();
            TELEMETRY.logUnitRun();
            for(Class<? extends Test> testClass : unitTests) {
                LOGGER.logSpecificRun(testClass.getName());
                TELEMETRY.clear();
                TELEMETRY.logSpecificRun(TestTelemetryLogger.TestType.UNIT, testClass.getName());
                OP_MODE.sleep(1000);
                HashMap<String, Boolean> result = new HashMap<>();
                long time = System.currentTimeMillis();
                try {
                    // Setup test (given), and fail the test if it threw an exception
                    UnitTest test = (UnitTest) testClass.newInstance();
                    try {
                        // Run test (when), and fail the test if it threw an exception
                        test.when();
                    } catch(Exception e) {
                        result.put(testClass.getName(), false);
                        failTest(test, time, FailureReason.EXCEPTION, e);
                    }
                    try {
                        // determine whether the test passed, and fail if if not or if it threw an exception
                        boolean grade = test.then();
                        if(grade) {
                            result.put(testClass.getName(), true);
                            passTest(test, time);
                        }else{
                            result.put(testClass.getName(), false);
                            failTest(test, time, FailureReason.FALSEHOOD, null);
                        }
                    } catch(Exception e) {
                        result.put(testClass.getName(), false);
                        failTest(test, time, FailureReason.EXCEPTION, e);
                    }
                } catch(Exception e) {
                    result.put(testClass.getName(), false);
                    failTest(testClass, time, e);
                }
                results.add(result);
            }
            OP_MODE.sleep(1000);
            LOGGER.logIntegrationRun();
            TELEMETRY.clear();
            TELEMETRY.logIntegrationRun();
            for(Class<? extends Test> testClass : integrationTests) {
                LOGGER.logSpecificRun(testClass.getName());
                TELEMETRY.clear();
                TELEMETRY.logSpecificRun(TestTelemetryLogger.TestType.UNIT, testClass.getName());
                OP_MODE.sleep(1000);
                HashMap<String, Boolean> result = new HashMap<>();
                long time = System.currentTimeMillis();
                try {
                    // Setup test (given), and fail the test if it threw an exception
                    IntegrationTest test = (IntegrationTest) testClass.newInstance();
                    try {
                        // Run test (when), and fail the test if it threw an exception
                        test.when();
                    } catch(Exception e) {
                        result.put(testClass.getName(), false);
                        failTest(test, time, FailureReason.EXCEPTION, e);
                    }
                    try {
                        // determine whether the test passed, and fail if if not or if it threw an exception
                        boolean grade = test.then();
                        if(grade) {
                            result.put(testClass.getName(), true);
                            passTest(test, time);
                        }else{
                            result.put(testClass.getName(), false);
                            failTest(test, time, FailureReason.FALSEHOOD, null);
                        }
                    } catch(Exception e) {
                        result.put(testClass.getName(), false);
                        failTest(test, time, FailureReason.EXCEPTION, e);
                    }
                } catch(Exception e) {
                    result.put(testClass.getName(), false);
                    failTest(testClass, time, e);
                }
                results.add(result);
            }
        }
        return results;
    }

    private void failTest(Class<? extends Test> test, long startTime, Exception e) {
        long endTime = System.currentTimeMillis();
        LOGGER.logTestResult(Grade.FAILING, test.getName(), "Failed at Initialization (then), logs unavailable.", startTime, endTime, FailureReason.EXCEPTION, e);
        TELEMETRY.logTestResult(Grade.FAILING, test.getName(), startTime, endTime, FailureReason.EXCEPTION, e);
        OP_MODE.sleep(2000);
        TELEMETRY.clear();
    }

    private void failTest(Test test, long startTime, FailureReason reason, Exception e) {
        long endTime = System.currentTimeMillis();
        LOGGER.logTestResult(Grade.FAILING, test.getClass().getName(), test.getLogs(), startTime, endTime, reason, e);
        TELEMETRY.logTestResult(Grade.FAILING, test.getName(), startTime, endTime, reason, e);
        OP_MODE.sleep(2000);
        TELEMETRY.clear();
    }

    private void passTest(Test test, long startTime) {
        long endTime = System.currentTimeMillis();
        LOGGER.logTestResult(Grade.PASSING, test.getClass().getName(), test.getLogs(), startTime, endTime, FailureReason.DID_NOT_FAIL, null);
        TELEMETRY.logTestResult(Grade.FAILING, test.getName(), startTime, endTime, FailureReason.DID_NOT_FAIL, null);
        OP_MODE.sleep(2000);
        TELEMETRY.clear();
    }

    private ArrayList<ArrayList<Class<? extends Test>>> findMeaningfulTests() {
        // find and merge tests from unit and integration packages
        Set<Class<?>> unit = findUnitTests();
        Set<Class<?>> integration = findIntegrationTests();
        ArrayList<Class<?>> allTests = new ArrayList<>();
        allTests.addAll(unit);
        allTests.addAll(integration);
        // define lists of tests
        ArrayList<ArrayList<Class<? extends Test>>> tests = new ArrayList<>();
        ArrayList<Class<? extends Test>> unitTests = new ArrayList<>();
        ArrayList<Class<? extends Test>> integrationTests = new ArrayList<>();
        // check if tests are unskippable and which type of test each one is, and assign each to correct list
        for(Class<?> test : allTests) {
            if(!test.isAnnotationPresent(Skippable.class) && UnitTest.class.isAssignableFrom(test)) {
                unitTests.add((Class<? extends Test>) test);
            }else if(!test.isAnnotationPresent(Skippable.class) && IntegrationTest.class.isAssignableFrom(test)) {
                integrationTests.add((Class<? extends Test>) test);
            }
        }
        // combine lists
        tests.add(unitTests);
        tests.add(integrationTests);
        return tests;
    }

    private Set<Class<?>> findUnitTests() {
        // find all class files in the unit package
        InputStream stream = ClassLoader.getSystemClassLoader().getResourceAsStream("org.firstinspires.ftc.teamcode.test.unit".replaceAll("[.]", File.pathSeparator));
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream));
        return reader.lines().filter(line -> line.endsWith(".class")).map(this::findUnitTest).collect(Collectors.toSet());
    }

    private Class<?> findUnitTest(String className) {
        // try to get a class via a class file from the unit test class files, and return a stub if failed
        try {
            return Class.forName("org.firstinspires.ftc.teamcode.test.unit" + "." + className.substring(0, className.lastIndexOf('.')));
        } catch (ClassNotFoundException e) {
            return StubUnitTest.class;
        }
    }

    private Set<Class<?>> findIntegrationTests() {
        // find all class files in integration package
        InputStream stream = ClassLoader.getSystemClassLoader().getResourceAsStream("org.firstinspires.ftc.teamcode.test.integration".replaceAll("[.]", File.pathSeparator));
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream));
        return reader.lines().filter(line -> line.endsWith(".class")).map(this::findIntegrationTest).collect(Collectors.toSet());
    }

    private Class<?> findIntegrationTest(String className) {
        // try to get a class via a class file from the integration test class files, and return a stub if failed
        try {
            return Class.forName("org.firstinspires.ftc.teamcode.test.integration" + "." + className.substring(0, className.lastIndexOf('.')));
        } catch (ClassNotFoundException e) {
            return StubIntegrationTest.class;
        }
    }

}
