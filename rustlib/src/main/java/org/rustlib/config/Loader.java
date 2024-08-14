package org.rustlib.config;

import android.os.Environment;

import org.rustlib.rustboard.RustboardServer;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.StringReader;
import java.io.StringWriter;

import javax.json.Json;
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

public class Loader {
    public static final File localStorage = new File(Environment.getExternalStorageDirectory().getPath(), "FIRST");
    public static final File externalStorage = Environment.getExternalStorageDirectory();

    public static String loadString(File file) {
        StringBuilder data = new StringBuilder();
        try (FileInputStream input = new FileInputStream(file)) {
            int character;
            while ((character = input.read()) != -1) {
                data.append((char) character);
            }

            return data.toString();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static String safeLoadString(File file, String defaultString) {
        try {
            return loadString(file);
        } catch (RuntimeException e) {
            return defaultString;
        }
    }

    public static String safeLoadString(File file) {
        return safeLoadString(file, "");
    }

    public static JsonObject loadJsonObject(File file) {
        return getJsonObject(loadString(file));
    }

    public static JsonObject safeLoadJsonObject(File file, JsonObject defaultJsonObject) {
        try {
            return loadJsonObject(file);
        } catch (RuntimeException e) {
            return defaultJsonObject;
        }
    }

    public static JsonObject safeLoadJsonObject(File file) {
        return safeLoadJsonObject(file, Json.createObjectBuilder().build());
    }

    public static String loadString(String filePath) {
        return loadString(new File(filePath));
    }

    public static String safeLoadString(String filePath, String defaultString) {
        try {
            return loadString(filePath);
        } catch (RuntimeException e) {
            return defaultString;
        }
    }

    public static String safeLoadString(String filePath) {
        return safeLoadString(filePath, "");
    }

    public static JsonObject loadJsonObject(String filePath) {
        return getJsonObject(loadString(filePath));
    }

    public static JsonObject safeLoadJsonObject(String filePath, JsonObject defaultJsonObject) {
        try {
            return loadJsonObject(filePath);
        } catch (RuntimeException e) {
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
            return Double.parseDouble(safeLoadString(file));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static double loadDouble(String filePath, double defaultValue) {
        return loadDouble(new File(filePath), defaultValue);
    }

    public static long loadLong(File file, long defaultValue) {
        try {
            return Long.parseLong(safeLoadString(file));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static long loadLong(String filePath, long defaultValue) {
        return loadLong(new File(filePath), defaultValue);
    }

    public static boolean loadBoolean(File file, boolean defaultValue) {
        try {
            return Boolean.parseBoolean(safeLoadString(file));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static boolean loadBoolean(String filePath, boolean defaultValue) {
        return loadBoolean(new File(filePath), defaultValue);
    }

    public static void writeString(File output, String string) throws IOException {
        FileOutputStream fileOut = new FileOutputStream(output.getAbsolutePath());
        try (OutputStreamWriter writer = new OutputStreamWriter(fileOut)) {
            writer.write(string);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        if (!output.exists()) {
            throw new RuntimeException("couldn't create file");
        }
    }

    public static void writeJson(File output, JsonObject json) throws IOException {
        writeString(output, json.toString());
    }

    public static void write(File output, Object o) throws IOException {
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
            write(output, o);
        } catch (IOException e) {
            RustboardServer.log(e);
        }
    }

    public static Document readXML(File file) {
        try {
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setValidating(true);
            factory.setIgnoringElementContentWhitespace(true);
            DocumentBuilder builder = factory.newDocumentBuilder();
            return builder.parse(file);
        } catch (IOException | SAXException | ParserConfigurationException e) {
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
}
