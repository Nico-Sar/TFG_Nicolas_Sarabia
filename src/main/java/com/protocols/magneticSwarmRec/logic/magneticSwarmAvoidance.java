package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.copter.Copter;
import com.api.copter.TakeOff;
import com.api.copter.TakeOffListener;
import com.api.pojo.location.Waypoint;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.Vector;
import com.uavController.UAVParam;
import es.upv.grc.mapper.Location2DGeo;
import es.upv.grc.mapper.Location3DGeo;
import es.upv.grc.mapper.Location3DUTM;
import net.objecthunter.exp4j.Expression;
import net.objecthunter.exp4j.ExpressionBuilder;
import org.javatuples.Pair;
import com.protocols.magneticSwarmRec.logic.DrawVectors;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class magneticSwarmAvoidance extends Thread {

    private Queue<Location3DUTM> waypoints;
    private final Copter copter;
    private final int numUAV;
    private final int numUAVs;
    private final Communication communication;
    private double minDistance = Double.MAX_VALUE;
    private final double maxspeed;
    private final DrawVectors drawVectors;

    private enum UAVState { MOVING, HOVERING, LANDING }
    private UAVState state = UAVState.MOVING;

    public magneticSwarmAvoidance(int numUAV) {
        this.numUAV = numUAV;
        this.numUAVs = API.getArduSim().getNumUAVs();
        this.copter = API.getCopter(numUAV);
        this.drawVectors = new DrawVectors(copter);
        this.communication = new Communication(numUAV);
        this.maxspeed = copter.getPlannedSpeed();
        setWaypoints();
    }

    private void setWaypoints() {
        waypoints = new LinkedList<>();
        List<Waypoint>[] missions = copter.getMissionHelper().getMissionsLoaded();
        for (int i = 1; i < missions[numUAV].size(); i++) {
            waypoints.add(new Location3DUTM(missions[numUAV].get(i).getUTM(), magneticSwarmRecSimProperties.altitude));
        }
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), copter.getAltitude());
    }

    private double getHeading() {
        return Math.toRadians(copter.getHeading());
    }

    @Override
    public void run() {
        takeoff();
        communication.start();
        long start = System.currentTimeMillis();
        API.getArduSim().sleep(2000);

        long reconfigStart = System.currentTimeMillis();
        switchToFlyingFormation();
        long reconfigEnd = System.currentTimeMillis();
        long reconfigDuration = reconfigEnd - reconfigStart;

        while (true) {
            switch (state) {
                case MOVING:
                    if (!waypoints.isEmpty() && !waypointReached()) {
                        Vector attraction = getAttractionVector();
                        Vector repulsion = calculateHybridRepulsion();

                        Vector finalVector = Vector.add(
                                attraction.scaledCopy(magneticSwarmRecSimProperties.weightAttraction),
                                repulsion.scaledCopy(magneticSwarmRecSimProperties.weightRepulsion));

                        drawVectors.update(attraction, repulsion, finalVector);
                        finalVector = reduceToMaxSpeed(finalVector);
                        moveUAV(finalVector);
                        logMinDistance();
                    } else if (!waypoints.isEmpty()) {
                        waypoints.poll();
                    } else {
                        state = UAVState.HOVERING;
                        communication.setHovering(true);
                    }
                    break;

                case HOVERING:
                    moveUAV(new Vector(0, 0));
                    communication.setHovering(true);
                    if (communication.allUAVsHovering()) {
                        state = UAVState.LANDING;
                    }
                    break;

                case LANDING:
                    communication.stopCommunication();
                    land();
                    long protocolTime = System.currentTimeMillis() - start;
                    saveData(protocolTime, reconfigDuration);
                    return;
            }
            API.getArduSim().sleep(100);
        }
    }


    private Vector calculateHybridRepulsion() {
        Vector totalRepulsion = new Vector();
        Vector headingVec = new Vector(Math.cos(getHeading()), Math.sin(getHeading()));

        double maxRepulsionMagnitude = maxspeed * 0.8;
        double cutoff = magneticSwarmRecSimProperties.frd;
        double mu = magneticSwarmRecSimProperties.dirFactor;
        double dirRatio = magneticSwarmRecSimProperties.dirRatio;

        List<Pair<Location3DUTM, Double>> obstacles = communication.getObstaclesWithHeading();

        System.out.printf("UAV %d - Detectados %d obstáculos\n", numUAV, obstacles.size());

        for (Pair<Location3DUTM, Double> obs : obstacles) {
            Location3DUTM obstacle = obs.getValue0();
            double dist = getCopterLocation().distance3D(obstacle);

            System.out.printf("UAV %d - Obstáculo a %.2f m\n", numUAV, dist);

            if (dist < cutoff) {
                Vector rep = new Vector(obstacle, getCopterLocation());
                rep.normalize();

                // Componente direccional
                double dot = rep.dot(headingVec);
                double angle = Math.acos(Math.min(Math.max(dot / (rep.magnitude() * headingVec.magnitude()), -1.0), 1.0));
                double directionFactor = Math.cos(mu * angle);
                if (directionFactor < 0) directionFactor = dirRatio;

                Vector directional = rep.scaledCopy(directionFactor);

                // Componente omnidireccional (γ)
                Vector omnidirectional = rep.scaledCopy(1.0);
                applyMagnitudeFunction(omnidirectional, dist);

                totalRepulsion = Vector.add(totalRepulsion, directional);
                totalRepulsion = Vector.add(totalRepulsion, omnidirectional);

                System.out.printf("UAV %d - Repulsión aplicada: dirección=%.2f, omni=(%.2f, %.2f)\n",
                        numUAV, directionFactor, omnidirectional.x, omnidirectional.y);
            } else {
                System.out.printf("UAV %d - Objeto ignorado (%.2f >= cutoff %.2f)\n", numUAV, dist, cutoff);
            }
        }

        if (totalRepulsion.magnitude() > maxRepulsionMagnitude) {
            totalRepulsion.normalize();
            totalRepulsion.scalarProduct(maxRepulsionMagnitude);
            System.out.printf("UAV %d - Repulsión limitada a %.2f m/s\n", numUAV, maxRepulsionMagnitude);
        }

        return totalRepulsion;
    }

    private void applyMagnitudeFunction(Vector repulsion, double distance) {
        try {
            Expression expr = new ExpressionBuilder(magneticSwarmRecSimProperties.repulsionMagnitude)
                    .variables("x", "frd", "a")
                    .build()
                    .setVariable("x", distance)
                    .setVariable("frd", magneticSwarmRecSimProperties.frd)
                    .setVariable("a", magneticSwarmRecSimProperties.alpha);

            double magnitude = expr.evaluate();
            repulsion.scalarProduct(magnitude);

            System.out.printf("UAV %d - Magnitud repulsiva evaluada: %.2f (distancia %.2f)\n", numUAV, magnitude, distance);
        } catch (Exception e) {
            System.err.printf("UAV %d - Error evaluando función de repulsión: %s\n", numUAV, e.getMessage());
        }
    }





    private Vector getAttractionVector() {
        Vector attraction = new Vector(getCopterLocation(), waypoints.peek());
        attraction.normalize();
        double dist = getCopterLocation().distance3D(waypoints.peek());
        attraction.scalarProduct(dist > 50 ? maxspeed : maxspeed / 2);
        return attraction;
    }

    private Vector reduceToMaxSpeed(Vector v) {
        if (v.magnitude() > maxspeed) {
            v.normalize();
            v.scalarProduct(maxspeed);
        }
        return v;
    }

    private void moveUAV(Vector resulting) {
        copter.moveTo(resulting.y, resulting.x, 0); // plano XY
    }

    private boolean waypointReached() {
        return getCopterLocation().distance3D(waypoints.peek()) < 5;
    }

    private void takeoff() {
        TakeOff takeOff = copter.takeOff(magneticSwarmRecSimProperties.altitude, new TakeOffListener() {
            @Override public void onCompleteActionPerformed() {}
            @Override public void onFailure() {}
        });
        takeOff.start();
        try {
            takeOff.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        waypoints.poll();
    }

    private void switchToFlyingFormation() {
        Formation target = FormationFactory.newFormation(
                Formation.Layout.valueOf(magneticSwarmRecSimProperties.flyingFormation.toUpperCase())
        );
        target.init(numUAVs, magneticSwarmRecSimProperties.flyingDistance);

        double alt = magneticSwarmRecSimProperties.altitude;

        // Centro promedio del enjambre actual
        double latSum = 0.0, lonSum = 0.0;
        for (int i = 0; i < numUAVs; i++) {
            latSum += API.getCopter(i).getLocation().getLatitude();
            lonSum += API.getCopter(i).getLocation().getLongitude();
        }

        double centerLat = latSum / numUAVs;
        double centerLon = lonSum / numUAVs;
        Location3DUTM centerUTM = Location3DGeo.getUTM(centerLat, centerLon, alt);

        Location3DUTM current = getCopterLocation();
        Location3DUTM targetPos = target.get3DUTMLocation(centerUTM, numUAV);

        // Interpolación lineal de trayectoria hacia la formación
        int steps = 3; // puedes ajustar la suavidad
        double dx = (targetPos.x - current.x) / steps;
        double dy = (targetPos.y - current.y) / steps;

        waypoints.clear();
        for (int i = 1; i <= steps; i++) {
            waypoints.add(new Location3DUTM(current.x + dx * i, current.y + dy * i, alt));
        }
    }


    private void land() {
        copter.land();
    }

    private void logMinDistance() {
        for (int i = 0; i < numUAVs; i++) {
            if (i == numUAV) continue;
            double dist = UAVParam.distances[numUAV][i].get();
            if (dist < minDistance) {
                minDistance = dist;
            }
        }
    }

    private void saveData(long protocolTime, long reconfigTime) {
        String fileName;
        if (magneticSwarmRecSimProperties.missionFile != null &&
                !magneticSwarmRecSimProperties.missionFile.isEmpty()) {
            fileName = magneticSwarmRecSimProperties.missionFile.get(0).getName().replace(".kml", "");
        } else {
            fileName = "default_mission";
        }

        File f = new File(fileName + "_results.csv");
        int battery = copter.getBattery();
        boolean includeHeader = !f.exists();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(f, true))) {
            if (includeHeader) {
                writer.write("numUAV,numUAVs,minDistance(m),protocolTime(ms),reconfigTime(ms),battery(%),beaconingTime(ms)\n");
            }

            writer.write(String.format("%d,%d,%.2f,%d,%d,%d,%d\n",
                    numUAV, numUAVs, minDistance, protocolTime, reconfigTime, battery,
                    magneticSwarmRecSimProperties.beaconingTime));

            // También mostramos por consola
            System.out.printf("UAV %d - Distancia mínima registrada: %.2f m\n", numUAV, minDistance);

            if (minDistance < magneticSwarmRecSimProperties.minFlightDistance) {
                System.err.printf("COLISIÓN: UAV %d registró distancia %.2f m (mínimo permitido %.2f m)\n",
                        numUAV, minDistance, magneticSwarmRecSimProperties.minFlightDistance);
            }

        } catch (IOException e) {
            System.err.println("Error guardando resultados CSV: " + e.getMessage());
        }
    }


}
