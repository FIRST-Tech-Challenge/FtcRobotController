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
    public static final File localStorage = new File("/sdcard/FIRST");
    public static final File defaultStorageDirectory = new File(Environment.getExternalStorageDirectory() + "\\Download");

    public static String loadString(File filePath) {
        StringBuilder data = new StringBuilder();
        try {
            FileInputStream input = new FileInputStream(filePath);

            int character;
            while ((character = input.read()) != -1) {
                data.append((char) character);
            }

            return data.toString();
        } catch (IOException e) {
            RustboardServer.log(e.toString());
            e.printStackTrace();
        }
        return "";
    }

    public static JsonObject loadJsonObject(File filePath) {
        return getJsonObject(loadString(filePath));
    }

    public static String loadString(String parentDir, String child, String fileExtension) {
        return loadString(new File(parentDir + "\\" + child + "." + fileExtension));
    }

    public static JsonObject loadJsonObject(String parentDir, String child, String fileExtension) {
        return getJsonObject(loadString(parentDir, child, fileExtension));
    }

    public static String loadString(String filePath, String fileExtension) {
        filePath = filePath.replace(" ", "_");
        return loadString(new File(filePath + "." + fileExtension));
    }

    public static JsonObject loadJsonObject(String filePath, String fileExtension) {
        return getJsonObject(loadString(filePath, fileExtension));
    }

    public static String loadString(String filePath) {
        return loadString(filePath, ".txt");
    }

    public static JsonObject loadJsonObject(String filePath) {
        return getJsonObject(loadString(filePath));
    }

    public static JsonObject getJsonObject(String jsonString) {
        return Json.createReader(new StringReader(jsonString)).readObject();
    }

    public static double loadNumber(File filePath, double defaultValue) {
        try {
            return Double.parseDouble(loadString(filePath));
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static double loadNumber(String parentDir, String child, String fileExtension, double defaultValue) {
        return loadNumber(parentDir + "/" + child + "." + fileExtension, defaultValue);
    }

    public static double loadNumber(String filePath, String fileExtension, double defaultValue) {
        return loadNumber(filePath + "." + fileExtension, defaultValue);
    }

    public static double loadNumber(String filePath, double defaultValue) {
        return loadNumber(new File(filePath), defaultValue);
    }

    public static void writeString(File output, String string) throws IOException {
        FileOutputStream fileOut = new FileOutputStream(output.getAbsolutePath());
        OutputStreamWriter writer = new OutputStreamWriter(fileOut);
        writer.write(string);
        writer.close();
    }

    public static Document readXML(File file) {
        try {
            DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
            factory.setValidating(true);
            factory.setIgnoringElementContentWhitespace(true);
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document document = builder.parse(file);
            return document;
        } catch (IOException | SAXException | ParserConfigurationException e) {
            throw new RuntimeException(e);
        }
    }

    public static void saveValue(JsonObject object, String fileExtension) throws IOException {
        JsonObject configuration = object.getJsonObject("configuration");
        String value = configuration.getString("input");
        String fileName = configuration.getString("id").replace(" ", "_") + "." + fileExtension;
        File output = new File(defaultStorageDirectory, fileName);
        Loader.writeString(output, value);
    }

    public static void saveValue(JsonObject object) throws IOException {
        saveValue(object, "txt");
    }

    public static void savePath(JsonObject object) throws IOException {
        saveValue(object, "json");
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
