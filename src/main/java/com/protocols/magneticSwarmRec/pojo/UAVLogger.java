package com.protocols.magneticSwarmRec.pojo;

import es.upv.grc.mapper.Location3DUTM;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class UAVLogger {
    private final List<String> lines = new ArrayList<>();
    private final int id;

    private long flightStartTime = -1;
    private long flightEndTime = -1;
    private long formationDuration = -1;  // ⏱️ Duración del cambio de formación

    public UAVLogger(int id) {
        this.id = id;
        lines.add("timestamp,posX,posY,posZ,velX,velY,velZ,distToTarget,repulsionMag,attractionMag,totalMag,jitterType,penaltyFactor,attempts,state");
    }

    public void log(double timestamp,
                    Location3DUTM pos,
                    double[] velocity,
                    double dist,
                    double repMag,
                    double attMag,
                    double totalMag,
                    String jitterType,
                    double penalty,
                    int attempts,
                    String state) {
        try {
            lines.add(String.format(Locale.US,
                    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.2f,%d,%s",
                    sanitize(timestamp), sanitize(pos.x), sanitize(pos.y), sanitize(pos.z),
                    sanitize(velocity[0]), sanitize(velocity[1]), sanitize(velocity[2]),
                    sanitize(dist), sanitize(repMag), sanitize(attMag), sanitize(totalMag),
                    jitterType, sanitize(penalty), attempts, state));
        } catch (Exception e) {
            System.err.printf("⚠️ Error al loguear datos del UAV %d: %s\n", id, e.getMessage());
        }
    }

    public void setFlightStartTime(long time) {
        this.flightStartTime = time;
    }

    public void setFlightEndTime(long time) {
        this.flightEndTime = time;
    }

    public void setFormationDuration(long duration) {
        this.formationDuration = duration;
    }

    public void saveToFile() {
        try {
            lines.add(""); // línea vacía
            lines.add("FlightStartTime(ms)," + flightStartTime);
            lines.add("FlightEndTime(ms)," + flightEndTime);
            if (flightStartTime >= 0 && flightEndTime >= 0) {
                lines.add("TotalFlightDuration(ms)," + (flightEndTime - flightStartTime));
            } else {
                lines.add("TotalFlightDuration(ms),-1");
            }

            lines.add("FormationChangeDuration(ms)," + formationDuration);

            Files.write(Paths.get("uav_" + id + "_log.csv"), lines);
        } catch (IOException e) {
            System.err.printf("❌ Error al guardar el log del UAV %d: %s\n", id, e.getMessage());
        }
    }

    private double sanitize(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
