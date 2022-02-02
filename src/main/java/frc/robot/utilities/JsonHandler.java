package frc.robot.utilities;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonSyntaxException;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.LocalConstants;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

//Static class, JsonHandler, uses gson to read and write the constants.json file
public class JsonHandler {
    private static final String path = Filesystem.getOperatingDirectory() + "/constants.json";
    private static final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    /**
     * Reads the constants.json file using gson and returns the constants object.
     * If the file does not exist, it generates it with blank values, and notifies the user.
     *
     * @return The constants object
     */
    public static LocalConstants getConstants() {
        LocalConstants constants = new LocalConstants();
        try {
            if(Files.exists(Paths.get(path))) {
                //read the file and write it back, in case the json is not valid.
                String json = Files.readString(Paths.get(path));
                constants = gson.fromJson(json, LocalConstants.class);
                System.out.println("Constants file read from " + path);
            } else {
                Files.createFile(Paths.get(path));
                System.out.println("Constants file not found, creating new file at " + path);
            }
        } catch(IOException e) {
            e.printStackTrace();
        } catch(JsonSyntaxException e) {
            System.out.println("Constants file is not valid JSON, creating new file at " + path);
            e.printStackTrace();
        }
        constants.write();
        return constants;
    }

    /**
     * Writes the constants to the constants.json file using gson
     *
     * @param constants The constants to write
     */
    public static void write(LocalConstants constants) {
        try {
            String json = gson.toJson(constants);
            //replace every newline with a ~
            json = json.replaceAll("\n", "~");
            //replace every two spaces with a tab
            json = json.replaceAll("\\s{2}", "\t");
            //replace every ~ with a newline
            json = json.replaceAll("~", "\n");
            Files.write(Path.of(path), json.getBytes(StandardCharsets.UTF_8));
            System.out.println("Constants file written to " + path);
        } catch(IOException e) {
            System.out.println("Error writing constants file to " + path);
            e.printStackTrace();
        }
    }

    /**
     * Writes the default constants to the constants.json file using gson
     */
    public static void write() {
        write(new LocalConstants());
    }
}