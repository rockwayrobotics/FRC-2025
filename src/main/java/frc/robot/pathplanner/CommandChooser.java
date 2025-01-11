package frc.robot.pathplanner;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FilenameFilter;
import java.util.HashMap;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandChooser {
  private HashMap<String, SendableChooser<String>> m_choosers = new HashMap<>();

  public CommandChooser() {
    try {
      File dir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
      File[] autolist = dir.listFiles(new FilenameFilter() {
        @Override
        public boolean accept(File dir, String name) {
          return name.endsWith(".auto");
        }
      });

      if (autolist != null) {
        for (var file : autolist) {
          String folderName = getFolderForAuto(file);
          if (folderName.isEmpty()) {
            // Ignore autos that aren't in a folder.
            continue;
          }

          String name = file.getName().replace(".auto", "");
          if (!m_choosers.containsKey(folderName)) {
            SendableChooser<String> chooser = new SendableChooser<>();
            chooser.setDefaultOption(name, name);
            m_choosers.put(folderName, chooser);
          } else {
            m_choosers.get(folderName).addOption(name, name);
          }
        }
      }
    } catch (Exception e) {
      System.out.println("Failed to read path planner folder structure: " + e.getMessage());
      e.printStackTrace();
    }
  }

  public HashMap<String, SendableChooser<String>> getChoosers() {
    return m_choosers;
  }

  private String getFolderForAuto(File file) {
    try (BufferedReader br = new BufferedReader(
        new FileReader(
            file))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
      var folder = json.get("folder");
      if (folder == null || !(folder instanceof String)) {
        return "";
      }
      return (String) folder;
    } catch (Exception e) {
      return "";
    }
  }

  public Command runAuto(String key) {
    var chooser = m_choosers.get(key);
    if (chooser == null) {
      System.out.println("No chooser found for key: " + key);
      return new Command() {
        // do nothing
      };
    }

    try {
      return new PathPlannerAuto(chooser.getSelected());
    } catch (Exception e) {
      System.out.println("Cannot run auto " + key + ":" + e.getStackTrace().toString());
      return new Command() {
        // do nothing
      };
    }
  }
}
