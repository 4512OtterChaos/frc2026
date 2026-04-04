package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.*;
import java.util.*;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/** A command that loads and runs an autonomous routine built using PathPlanner. */
public class OCPathPlannerAuto extends PathPlannerAuto {
  /** The currently running path name. Used to handle activePath triggers */
  public static Optional<PathPlannerPath> currentOrLastPath = Optional.empty();

  /**
   * Constructs a new PathPlannerAuto command.
   *
   * @param autoName the name of the autonomous routine to load and run
   * @param mirror Mirror all paths to the other side of the current alliance. For example, if a
   *     path is on the right of the blue alliance side of the field, it will be mirrored to the
   *     left of the blue alliance side of the field.
   * @throws AutoBuilderException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine
   */
  public static OCPathPlannerAuto buildAuto(String autoName) {
    return buildAuto(autoName, false);
  }
  public static OCPathPlannerAuto buildAuto(String autoName, boolean mirror) {
    if (!AutoBuilder.isConfigured()) {
      throw new AutoBuilderException(
          "AutoBuilder was not configured before attempting to load a PathPlannerAuto from file");
    }

    Command autoCommand;
    Pose2d startingPose;

    try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

      String version = json.get("version").toString();
      String[] versions = version.split("\\.");

      if (!versions[0].equals("2025")) {
        throw new FileVersionException(version, "2025.X", autoName + ".auto");
      }

      var setup = initFromJson(json, mirror);
      autoCommand = setup.getFirst();
      startingPose = setup.getSecond();
    } catch (FileNotFoundException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
      autoCommand = Commands.none();
      startingPose = new Pose2d();
    } catch (IOException e) {
      DriverStation.reportError(
          "Failed to read file required by auto: " + autoName, e.getStackTrace());
      autoCommand = Commands.none();
      startingPose = new Pose2d();
    } catch (ParseException e) {
      DriverStation.reportError(
          "Failed to parse JSON in file required by auto: " + autoName, e.getStackTrace());
      autoCommand = Commands.none();
      startingPose = new Pose2d();
    } catch (FileVersionException e) {
      DriverStation.reportError(
          "Failed to load auto: " + autoName + ". " + e.getMessage(), e.getStackTrace());
      autoCommand = Commands.none();
      startingPose = new Pose2d();
    }
    return new OCPathPlannerAuto(autoCommand, startingPose);
  }

  /**
   * Create a PathPlannerAuto from a custom command
   *
   * @param autoCommand The command this auto should run
   * @param startingPose The starting pose of the auto. Only used for the getStartingPose method
   */
  private OCPathPlannerAuto(Command autoCommand, Pose2d startingPose) {
    super(autoCommand, startingPose);
  }


  /**
   * Create a trigger that is high when a certain path is being followed
   *
   * @param pathName The name of the path to check for
   * @return activePath trigger
   */
  public Trigger activePath(String pathName) {
    return condition(() -> pathName.equals(currentPathName));
  }

  private static Pair<Command, Pose2d> initFromJson(JSONObject autoJson, boolean mirror)
      throws IOException, ParseException, FileVersionException {
    boolean choreoAuto = autoJson.get("choreoAuto") != null && (boolean) autoJson.get("choreoAuto");
    JSONObject commandJson = (JSONObject) autoJson.get("command");
    Command cmd = OCCommandUtil.commandFromJson(commandJson, choreoAuto, mirror);
    Pose2d startingPose;
    boolean resetOdom = autoJson.get("resetOdom") != null && (boolean) autoJson.get("resetOdom");
    List<PathPlannerPath> pathsInAuto = pathsFromCommandJson(commandJson, choreoAuto);
    if (!pathsInAuto.isEmpty()) {
      PathPlannerPath path0 = pathsInAuto.get(0);
      if (mirror) {
        path0 = path0.mirrorPath();
      }
      if (AutoBuilder.isHolonomic()) {
        startingPose =
            new Pose2d(path0.getPoint(0).position, path0.getIdealStartingState().rotation());
      } else {
        startingPose = path0.getStartingDifferentialPose();
      }
    } else {
      startingPose = null;
    }

    if (resetOdom) {
      cmd = Commands.sequence(AutoBuilder.resetOdom(startingPose), cmd);
    }

    return new Pair<Command,Pose2d>(cmd, startingPose);
  }

  private static List<PathPlannerPath> pathsFromCommandJson(
      JSONObject commandJson, boolean choreoPaths) throws IOException, ParseException {
    List<PathPlannerPath> paths = new ArrayList<>();

    String type = (String) commandJson.get("type");
    JSONObject data = (JSONObject) commandJson.get("data");

    if (type.equals("path")) {
      String pathName = (String) data.get("pathName");
      if (choreoPaths) {
        paths.add(PathPlannerPath.fromChoreoTrajectory(pathName));
      } else {
        paths.add(PathPlannerPath.fromPathFile(pathName));
      }
    } else if (type.equals("sequential")
        || type.equals("parallel")
        || type.equals("race")
        || type.equals("deadline")) {
      for (var cmdJson : (JSONArray) data.get("commands")) {
        paths.addAll(pathsFromCommandJson((JSONObject) cmdJson, choreoPaths));
      }
    }

    return paths;
  }
}
