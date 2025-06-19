package com.protocols.magneticSwarmRec.gui;

import com.api.API;
import com.api.ArduSimTools;
import com.api.MissionHelper;
import com.api.pojo.location.Waypoint;
import com.setup.Text;
import es.upv.grc.mapper.Location2DGeo;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.util.EnumValue;
import org.javatuples.Pair;

import java.io.File;
import java.lang.reflect.Field;
import java.util.*;

public class magneticSwarmRecSimProperties {

    // === Parámetros configurables desde el FXML ===
    public static double speed = 5.0;
    public static double threshold = 2.5;
    public static double minFlightDistance = 10.0;
    public static double frdOmni = 15.0;
    public static double frdDir = 30.0;
    public static double maxRepulsionFactor = 0.6;
    public static double omniRepulsionStrength = 2.0;
    public static double dirRepulsionStrength = 0.7;
    public static double dirFactor = 2.5;
    public static double weightAttraction = 1.0;
    public static double weightRepulsion = 1.0;
    public static double angleWeight = 30.0;
    public static double neighborDetectionRadius = 25.0;

    public static String groundFormation = "LINEAR";
    public static String flyingFormation = "MATRIX";
    public static String configMode = "FORMATION";


    public static double groundDistance = 50.0;
    public static double flyingDistance = 50.0;

    // === Parámetros adicionales internos ===
    public static double altitude = 10;
    public static boolean randomPath = false;
    public static int seed = 42;
    public static double SWlat, SWlon, NElat, NElon;
    public static List<File> missionFile;

    private Random random;

    public boolean storeParameters(Properties guiParams, ResourceBundle fileParams) {
        Properties parameters = new Properties();
        for (String key : fileParams.keySet()) {
            parameters.setProperty(key, guiParams.containsKey(key) ? guiParams.getProperty(key) : fileParams.getString(key));
        }

        Field[] variables = this.getClass().getDeclaredFields();
        Map<String, Field> variablesDict = new HashMap<>();
        for (Field var : variables) {
            variablesDict.put(var.getName(), var);
        }

        for (Object keyObj : parameters.keySet()) {
            String key = keyObj.toString();
            String value = parameters.getProperty(key);
            if (!variablesDict.containsKey(key)) continue;

            Field var = variablesDict.get(key);
            try {
                if (value == null || value.trim().isEmpty()) continue;
                String type = var.getType().toString();

                if (key.equals("groundFormation") || key.equals("flyingFormation")) {
                    var.set(this, value.trim().toUpperCase());
                    continue;
                }

                if (type.equals("int")) {
                    var.setInt(this, Integer.parseInt(value));
                } else if (type.equals("double")) {
                    var.setDouble(this, Double.parseDouble(value));
                } else if (type.contains("java.lang.String")) {
                    var.set(this, value);
                } else if (type.contains("boolean")) {
                    var.set(this, Boolean.parseBoolean(value));
                } else if (type.contains("java.util.List")) {
                    String[] filesNames = value.split(";");
                    List<File> files = new ArrayList<>();
                    for (String fileName : filesNames) {
                        fileName = API.getFileTools().getResourceFolder().toString() + File.separator + fileName;
                        File f = new File(fileName);
                        String extension = fileName.substring(fileName.lastIndexOf('.') + 1);
                        if (f.exists() && (extension.equals(Text.FILE_EXTENSION_WAYPOINTS) || extension.equals(Text.FILE_EXTENSION_KML))) {
                            files.add(f);
                        }
                    }
                    var.set(this, files);
                } else {
                    ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_STORE_PARAMETERS + type);
                    return false;
                }

            } catch (IllegalAccessException e) {
                return false;
            }
        }

        String error = specificCheckVariables();
        if (error == null) {
            return true;
        } else {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, "Error in parameter: " + error);
            return false;
        }
    }

    private String specificCheckVariables() {
        if (!randomPath) {
            if (missionFile == null || missionFile.isEmpty()) return "missionFile is empty";
            for (File f : missionFile) {
                if (!f.exists()) return "missionFile " + f.getName() + " does not exist";
            }
        }

        if (frdOmni < 0 || frdDir < 0 || maxRepulsionFactor < 0 || dirFactor < 0)
            return "Repulsion parameters must be non-negative";
        if (speed <= 0) return "speed must be positive";
        if (groundDistance <= 0 || flyingDistance <= 0) return "formation distances must be positive";

        int numUAVs = API.getArduSim().getNumUAVs();
        if (numUAVs < 1) return "Invalid number of UAVs from GUI";

        Set<String> validFormations = Set.of("LINEAR", "MATRIX", "CIRCLE");
        if (!validFormations.contains(groundFormation)) return "Invalid groundFormation: " + groundFormation;
        if (!validFormations.contains(flyingFormation)) return "Invalid flyingFormation: " + flyingFormation;

        return null;
    }

    public void setSimulationParameters() {
        Pair<String, List<Waypoint>[]> missions = randomPath ? setMissionWaypoints() : API.getGUI(0).loadMissions(missionFile);
        MissionHelper missionHelper = API.getCopter(0).getMissionHelper();
        missionHelper.setMissionsLoaded(missions.getValue1());
    }

    private Pair<String, List<Waypoint>[]> setMissionWaypoints() {
        random = new Random(seed);
        int count = API.getArduSim().getNumUAVs();
        MissionHelper missionHelper = API.getCopter(0).getMissionHelper();

        EnumValue<MavCmd> cmd_waypoint = EnumValue.of(MavCmd.MAV_CMD_NAV_WAYPOINT);
        EnumValue<MavCmd> cmd_takeoff = EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF);
        EnumValue<MavCmd> cmd_land = EnumValue.of(MavCmd.MAV_CMD_NAV_LAND);

        Waypoint start = new Waypoint(0, true, MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, cmd_waypoint,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1);

        List<Waypoint>[] al = new ArrayList[count];
        List<Location2DGeo> initialPositions = new ArrayList<>();

        for (int i = 0; i < count; i++) {
            al[i] = new ArrayList<>();
            al[i].add(start);

            Location2DGeo l1 = null;
            while (l1 == null) {
                l1 = getRandomLocation();
                boolean valid = true;
                for (Location2DGeo ip : initialPositions) {
                    if (l1.getUTM().distance(ip.getUTM()) < minFlightDistance) {
                        valid = false;
                        break;
                    }
                }
                if (!valid) l1 = null;
            }

            initialPositions.add(l1);

            Location2DGeo l2 = null;
            while (l2 == null) {
                l2 = getRandomLocation();
                if (l1.getUTM().distance(l2.getUTM()) < minFlightDistance) {
                    l2 = null;
                }
            }

            al[i].add(new Waypoint(1, false, MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, cmd_takeoff,
                    0.0, 0.0, 0.0, 0.0,
                    l1.latitude, l1.longitude, altitude, 1));

            al[i].add(new Waypoint(2, false, MavFrame.MAV_FRAME_GLOBAL, cmd_waypoint,
                    0.0, 0.0, 0.0, 0.0,
                    l2.latitude, l2.longitude, altitude, 1));

            al[i].add(new Waypoint(3, false, MavFrame.MAV_FRAME_GLOBAL, cmd_land,
                    0.0, 0.0, 0.0, 0.0,
                    l2.latitude, l2.longitude, 30.0, 0));
        }

        missionHelper.setMissionsLoaded(al);
        return new Pair<>("generatedPath", al);
    }

    private Location2DGeo getRandomLocation() {
        double lat = SWlat + (NElat - SWlat) * random.nextDouble();
        double lon = SWlon + (NElon - SWlon) * random.nextDouble();
        return new Location2DGeo(lat, lon);
    }
}
