package org.firstinspires.ftc.teamcode.sample;

import static org.junit.jupiter.api.Assertions.*;

import android.annotation.SuppressLint;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.firebase.crashlytics.buildtools.reloc.org.apache.commons.io.IOUtils;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Paths;


class SampleTest {

    class MockLLResult extends LLResult {
        public MockLLResult(JSONObject json) throws JSONException {
            super(json);
        }
    }
    public static JSONObject parseJSONFile(String filename) throws JSONException, IOException {
        @SuppressLint("NewApi") String content = new String(Files.readAllBytes(Paths.get(filename)));
        return new JSONObject(content);
    }

    public void testWithJsonFile(String filename, String expected) throws IOException, JSONException {
        // Read JSON file
        JSONObject json = parseJSONFile("src/main/java/org/firstinspires/ftc/teamcode/sample/test_data/" + filename);
        MockLLResult llResult = new MockLLResult(json);

        // Create Sample instance
        Sample sample = new Sample(llResult);

        // Perform assertions
        assertEquals(expected, sample.toString());
    }

    @org.junit.jupiter.api.Test
    void testInvalidSmallTa() throws IOException, JSONException {
        testWithJsonFile("invalid_small_ta.json",
                "Invalid LLResult with error code -10032");
    }
    @org.junit.jupiter.api.Test
    void testValid1() throws IOException, JSONException {
        testWithJsonFile("valid_1.json",
                "Sample{x=7.7890026335380895, y=-2.812398678830519, d=1.1396787145269092, a=-38, c=Red}");
    }
    @org.junit.jupiter.api.Test
    void testValid2() throws IOException, JSONException {
        testWithJsonFile("valid_2.json",
                "Sample{x=-11.301597643280246, y=-5.049326353835767, d=1.2806063514362331, a=-79, c=Shared}");
    }
    @org.junit.jupiter.api.Test
    void testValid3() throws IOException, JSONException {
        testWithJsonFile("valid_3.json",
                "Sample{x=2.1398941752225795, y=4.549280178549225, d=0.8141990767695284, a=40, c=Blue}");
    }
    @org.junit.jupiter.api.Test
    void testValid4() throws IOException, JSONException {
        testWithJsonFile("valid_4.json",
                "Sample{x=0.8804047987464685, y=1.5270221743177501, d=0.5545438298536771, a=13, c=Unknown}");
    }
}