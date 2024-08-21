package org.rustlib.utils;

import android.os.Environment;

import org.rustlib.rustboard.RustboardServer;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.RandomAccessFile;
import java.io.StringReader;
import java.io.StringWriter;

import javax.json.Json;
import javax.json.JsonException;
import javax.json.JsonObject;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

public class FileUtils {
    public static final File localStorage = new File(Environment.getExternalStorageDirectory().getPath(), "FIRST");
    public static final File externalStorage = Environment.getExternalStorageDirectory();

    public static byte[] readBytes(File file) throws IOException {
        try (RandomAccessFile ramFile = new RandomAccessFile(file, "r")) {
            byte[] fileBytes = new byte[(int) ramFile.length()];
            ramFile.readFully(fileBytes);
            return fileBytes;
        }
    }

    public static byte[] safeReadBytes(File file, byte[] defaultContent) {
        try {
            return readBytes(file);
        } catch (IOException e) {
            return defaultContent;
        }
    }

    public static byte[] safeReadBytes(File file) {
        return safeReadBytes(file, new byte[0]);
    }

    public static String readString(File file) throws IOException {
        StringBuilder content = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String currentLine;
            while ((currentLine = reader.readLine()) != null) {
                content.append(currentLine).append(System.lineSeparator());
            }
        }
        return content.toString();
    }

    public static String safeReadString(File file, String defaultString) {
        try {
            return readString(file);
        } catch (IOException e) {
            return defaultString;
        }
    }

    public static String safeReadString(File file) {
        return safeReadString(file, "");
    }

    public static JsonObject loadJsonObject(File file) throws IOException {
        return getJsonObject(readString(file));
    }

    public static JsonObject safeLoadJsonObject(File file, JsonObject defaultJsonObject) {
        try {
            return loadJsonObject(file);
        } catch (IOException | JsonException e) {
            return defaultJsonObject;
        }
    }

    public static JsonObject safeLoadJsonObject(File file) {
        return safeLoadJsonObject(file, Json.createObjectBuilder().build());
    }

    public static String readString(String filePath) throws IOException {
        return readString(new File(filePath));
    }

    public static String safeReadString(String filePath, String defaultString) {
        try {
            return readString(filePath);
        } catch (IOException e) {
            return defaultString;
        }
    }

    public static String safeReadString(String filePath) {
        return safeReadString(filePath, "");
    }

    public static JsonObject loadJsonObject(String filePath) throws IOException {
        return getJsonObject(readString(filePath));
    }

    public static JsonObject safeLoadJsonObject(String filePath, JsonObject defaultJsonObject) {
        try {
            return loadJsonObject(filePath);
        } catch (IOException e) {
            return defaultJsonObject;
        }
    }

    public static JsonObject safeLoadJsonObject(String filePath) {
        return safeLoadJsonObject(filePath, Json.createObjectBuilder().build());
    }

    public static JsonObject getJsonObject(String jsonString) {
        return Json.createReader(new StringReader(jsonString)).readObject();
    }

    public static double loadDouble(File file, double defaultValue) {
        try {
            return Double.parseDouble(safeReadString(file));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static double loadDouble(String filePath, double defaultValue) {
        return loadDouble(new File(filePath), defaultValue);
    }

    public static long loadLong(File file, long defaultValue) {
        try {
            return Long.parseLong(safeReadString(file));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static long loadLong(String filePath, long defaultValue) {
        return loadLong(new File(filePath), defaultValue);
    }

    public static boolean loadBoolean(File file, boolean defaultValue) {
        try {
            return Boolean.parseBoolean(safeReadString(file));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static boolean loadBoolean(String filePath, boolean defaultValue) {
        return loadBoolean(new File(filePath), defaultValue);
    }

    public static void writeBytes(File output, byte[] bytes) throws IOException {
        try (FileOutputStream fileOut = new FileOutputStream(output.getAbsolutePath())) {
            fileOut.write(bytes);
        }
    }

    public static void safeWriteBytes(File output, byte[] bytes) {
        try {
            writeBytes(output, bytes);
        } catch (IOException e) {
            RustboardServer.log(e);
        }
    }

    public static void writeString(File output, String string) throws IOException {
        try (FileOutputStream fileOut = new FileOutputStream(output.getAbsolutePath()); OutputStreamWriter writer = new OutputStreamWriter(fileOut)) {
            writer.write(string);
        }
    }

    public static void writeJson(File output, JsonObject json) throws IOException {
        writeString(output, json.toString());
    }

    public static void writeObjectToString(File output, Object o) throws IOException {
        writeString(output, o.toString());
    }

    public static void safeWriteString(File output, String string) {
        try {
            writeString(output, string);
        } catch (IOException e) {
            RustboardServer.log(e);
        }
    }

    public static void safeWriteJson(File output, JsonObject json) {
        try {
            writeJson(output, json);
        } catch (IOException e) {
            RustboardServer.log(e);
        }
    }

    public static void safeWrite(File output, Object o) {
        try {
            writeObjectToString(output, o);
        } catch (IOException e) {
            RustboardServer.log(e);
        }
    }

    public static void copyFile(File source, File target) throws IOException {
        byte[] contents = readBytes(source);
        writeBytes(target, contents);
    }

    public static void safeCopyFile(File source, File target) {
        try {
            copyFile(source, target);
        } catch (IOException e) {
            RustboardServer.log(e);
        }
    }

    public static void clearDir(File dir, boolean recurse) throws IOException {
        File[] files = dir.listFiles();
        if (files != null) {
            for (File file : files) {
                if (file.isFile()) {
                    if (!file.delete()) {
                        throw new IOException(String.format("Could not delete '%s'", file.getAbsolutePath()));
                    }
                } else if (file.isDirectory() && recurse) {
                    clearDir(file, true);
                }
            }
        }
    }

    public static void clearDir(File dir) throws IOException {
        clearDir(dir, false);
    }

    public static void safeClearDir(File dir, boolean recurse) {
        File[] files = dir.listFiles();
        if (files != null) {
            for (File file : files) {
                if (file.isFile()) {
                    file.delete();
                } else if (file.isDirectory() && recurse) {
                    safeClearDir(file, true);
                }
            }
        }
    }

    public static void safeClearDir(File dir) {
        safeClearDir(dir, false);
    }

    public static Document readXML(File file) throws IOException {
        try {
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setValidating(true);
            factory.setIgnoringElementContentWhitespace(true);
            DocumentBuilder builder = factory.newDocumentBuilder();
            return builder.parse(file);
        } catch (SAXException | ParserConfigurationException e) {
            throw new RuntimeException(e);
        }
    }

    public static JsonObject readJsonString(String json) {
        return Json.createReader(new StringReader(json)).readObject();
    }

    public static String toXMLString(Document document) throws TransformerException, IOException {
        Transformer transformer = TransformerFactory.newInstance().newTransformer();
        transformer.setOutputProperty(OutputKeys.METHOD, "xml");
        transformer.setOutputProperty(OutputKeys.INDENT, "yes");
        transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", Integer.toString(2));

        StringWriter stringWriter = new StringWriter();
        StreamResult result = new StreamResult(stringWriter);
        DOMSource source = new DOMSource(document.getDocumentElement());

        transformer.transform(source, result);
        return stringWriter.toString();
    }

    public static void makeDirIfMissing(File file) {
        if (!file.exists()) {
            if (!file.mkdir()) {
                throw new RuntimeException(String.format("Could not make directory '%s'", file.getPath()));
            }
        }
    }
}
