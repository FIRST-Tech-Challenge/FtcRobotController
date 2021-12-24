package org.firstinspires.ftc.teamcode.main.utils.tests;

import java.io.BufferedReader;
import java.io.File;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;
import java.util.Set;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.stream.Collectors;
import java.util.zip.DataFormatException;

/**
 * A Tester finds all Tests and tests them automatically, unless they have an @Skippable annotation. If a Test throws an Exception or Test.then() returns false, the Test fails. If Test.then() return true, the Test passes.
 */

public class Tester {

    private final TestLogger LOGGER;

    /**
     * Creates a new Tester, which will run all Tests it can find automatically.
     */
    public Tester() {
        LOGGER = new TestLogger();
        LOGGER.logStart();
        LOGGER.logSearch();
        ArrayList<ArrayList<Class<? extends Test>>> tests = findMeaningfulTests();
        try {
            runTests(tests);
        } catch(DataFormatException e) {
            LOGGER.logFindErr();
        }
        LOGGER.logEnd();
        // TODO: telemetry logging
        // TODO: virtualize robot - in the sense of being able to run an opmode in an android emulator
    }

    private void runTests(ArrayList<ArrayList<Class<? extends Test>>> tests) throws DataFormatException {
        if(tests.size() != 2) {
            // fail if test arraylist is formatted incorrectly
            throw new DataFormatException("Test Arraylist must only have two elements.");
        }else{
            LOGGER.logFind();
            // get unit/integration tests from where they're expected to be and run each test
            ArrayList<Class<? extends Test>> unitTests = tests.get(0);
            ArrayList<Class<? extends Test>> integrationTests = tests.get(1);
            LOGGER.logUnitRun();
            for(Class<? extends Test> testClass : unitTests) {
                LOGGER.logSpecificRun(testClass.getName());
                long time = System.currentTimeMillis();
                try {
                    // Setup test (given), and fail the test if it threw an exception
                    UnitTest test = (UnitTest) testClass.newInstance();
                    try {
                        // Run test (when), and fail the test if it threw an exception
                        test.when();
                    } catch(Exception e) {
                        failTest(test, time);
                    }
                    try {
                        // determine whether the test passed, and fail if if not or if it threw an exception
                        boolean grade = test.then();
                        if(grade) {
                            passTest(test, time);
                        }else{
                            failTest(test, time);
                        }
                    } catch(Exception e) {
                        failTest(test, time);
                    }
                } catch(Exception e) {
                    failTest(testClass, time);
                }
            }
            LOGGER.logIntegrationRun();
            for(Class<? extends Test> testClass : integrationTests) {
                LOGGER.logSpecificRun(testClass.getName());
                long time = System.currentTimeMillis();
                try {
                    // Setup test (given), and fail the test if it threw an exception
                    IntegrationTest test = (IntegrationTest) testClass.newInstance();
                    try {
                        // Run test (when), and fail the test if it threw an exception
                        test.when();
                    } catch(Exception e) {
                        failTest(test, time);
                    }
                    try {
                        // determine whether the test passed, and fail if if not or if it threw an exception
                        boolean grade = test.then();
                        if(grade) {
                            passTest(test, time);
                        }else{
                            failTest(test, time);
                        }
                    } catch(Exception e) {
                        failTest(test, time);
                    }
                } catch(Exception e) {
                    failTest(testClass, time);
                }
            }
        }
    }

    private void failTest(Class<? extends Test> test, long startTime) {
        LOGGER.logTestResult(TestLogger.Grade.FAILING, test.getName(), "Failed at Initialization (then), logs unavailable.", startTime, System.currentTimeMillis());
    }

    private void failTest(Test test, long startTime) {
        LOGGER.logTestResult(TestLogger.Grade.FAILING, test.getClass().getName(), test.getLogs(), startTime, System.currentTimeMillis());
    }

    private void passTest(Test test, long startTime) {
        LOGGER.logTestResult(TestLogger.Grade.PASSING, test.getClass().getName(), test.getLogs(), startTime, System.currentTimeMillis());
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
