package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.copter.Copter;
import com.api.copter.TakeOff;
import com.api.copter.TakeOffListener;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.HungarianAlgorithm;
import com.protocols.magneticSwarmRec.pojo.UAVLogger;
import com.protocols.magneticSwarmRec.pojo.Vector;
import es.upv.grc.mapper.Location3DGeo;
import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;

import java.util.*;
import java.util.concurrent.CyclicBarrier;

public class magneticSwarmAvoidance extends Thread {
    private Queue<Location3DUTM> waypoints;
    private final Copter copter;
    private final int numUAV, numUAVs;
    private final Communication communication;
    private final double maxspeed;
    private final DrawVectors drawVectors;
    private static CyclicBarrier barrier, landingBarrier;
    private int attempts = 0;
    private final double threshold = 2.5;
    private UAVState state = UAVState.MOVING;
    private Location3DUTM centerUTM, finalTarget;
    private enum UAVState { MOVING, HOVERING, LANDING }
    private final UAVLogger logger;
    private long formationStartTime = -1;

    private static final boolean debugFormacion = true;

    public magneticSwarmAvoidance(int numUAV) {
        this.numUAV = numUAV;
        this.numUAVs = API.getArduSim().getNumUAVs();
        this.copter = API.getCopter(numUAV);
        this.drawVectors = new DrawVectors(copter);
        this.communication = new Communication(numUAV);
        this.maxspeed = copter.getPlannedSpeed();
        this.waypoints = new LinkedList<>();
        this.logger = new UAVLogger(numUAV);
        synchronized (magneticSwarmAvoidance.class) {
            if (barrier == null) {
                barrier = new CyclicBarrier(numUAVs);
                landingBarrier = new CyclicBarrier(numUAVs);
            }
        }
    }

    @Override
    public void run() {
        takeoff();
        communication.start();
        switchToFlyingFormation();
        formationStartTime = System.currentTimeMillis();
        while (true) {
            switch (state) {
                case MOVING:
                    if (waypoints.isEmpty()) {
                        state = UAVState.HOVERING;
                        break;
                    }

                    Location3DUTM destino = waypoints.peek();
                    Location3DUTM current = getCopterLocation();
                    double dist = current.distance3D(destino);

                    Vector attraction = getAttractionVector();
                    Vector repulsion = calculateHybridRepulsion();
                    penalizeAttractionIfNeeded(attraction, repulsion, dist);

                    Vector total = Vector.add(attraction, repulsion);
                    checkPotentialCollision(total);
                    sendVelocityCommand(total);

                    long timestamp = System.currentTimeMillis() - API.getArduSim().getExperimentStartTime();
                    double[] vel = copter.getSpeedComponents();

                    String jitterType = "none";
                    if (total.magnitude() < 0.2 && attempts > 30) {
                        jitterType = "soft";
                    } else if (total.magnitude() < 0.5 && current.distance3D(getLastTarget()) < 6.0) {
                        jitterType = "escape";
                    }

                    double penalty = (repulsion.magnitude() > 0.2 * maxspeed && dist > 5.0)
                            ? Math.max(0.5, 1.0 - repulsion.magnitude() / maxspeed)
                            : 1.0;

                    logger.log(
                            timestamp,
                            current,
                            vel,
                            dist,
                            repulsion.magnitude(),
                            attraction.magnitude(),
                            total.magnitude(),
                            jitterType,
                            penalty,
                            attempts,
                            state.name()
                    );

                    if (dist < threshold && isUAVStable()) {
                        waypoints.poll();
                        attempts = 0;
                    } else if (dist <= 3.0 && attempts > 60) {
                        System.out.printf("⚠️ UAV %d está cerca (%.2f m) pero aún se mueve\n", numUAV, dist);
                        waypoints.poll();
                        attempts = 0;
                    } else {
                        attempts++;
                    }
                    break;

                case HOVERING:
                    moveUAV(new Vector(0, 0));
                    communication.setHovering(true);
                    if (communication.allUAVsHovering()) {
                        try {
                            System.out.printf("🛑 UAV %d en posición final. Esperando para aterrizar...\n", numUAV);
                            API.getArduSim().sleep(3000);
                            landingBarrier.await();
                            logger.setFlightEndTime(System.currentTimeMillis());
                            logger.setFormationDuration(System.currentTimeMillis() - formationStartTime);
                            state = UAVState.LANDING;
                        } catch (Exception ignored) {}
                    }
                    break;

                case LANDING:
                    communication.stopCommunication();
                    Location3DUTM finalPos = getCopterLocation();
                    System.out.printf("✅ UAV %d aterriza en (%.2f, %.2f) | Distancia al objetivo = %.2f\n",
                            numUAV, finalPos.x, finalPos.y, finalPos.distance3D(finalTarget));
                    copter.land();
                    logger.saveToFile();
                    return;
            }
            API.getArduSim().sleep(100);
        }
    }

    private void checkPotentialCollision(Vector velocity) {
        for (Pair<Location3DUTM, Double> obs : communication.getObstaclesWithHeading()) {
            Location3DUTM other = obs.getValue0();
            double dist = getCopterLocation().distance3D(other);
            if (dist < 3.0) {
                System.out.printf("❌ Posible colisión: UAV %d muy cerca de otro UAV (%.2f m)\n", numUAV, dist);
            }
        }
    }

    private void takeoff() {
        TakeOff takeOff = copter.takeOff(magneticSwarmRecSimProperties.altitude, new TakeOffListener() {
            public void onCompleteActionPerformed() {}
            public void onFailure() {}
        });
        takeOff.start();
        try {
            takeOff.join();
            barrier.await();
            logger.setFlightStartTime(System.currentTimeMillis());
        } catch (Exception ignored) {}
    }

    private void switchToFlyingFormation() {
        Formation target = FormationFactory.newFormation(
                Formation.Layout.valueOf(magneticSwarmRecSimProperties.flyingFormation.toUpperCase())
        );
        target.init(numUAVs, magneticSwarmRecSimProperties.flyingDistance);
        double alt = magneticSwarmRecSimProperties.altitude;

        double latSum = 0.0, lonSum = 0.0;
        for (int i = 0; i < numUAVs; i++) {
            latSum += API.getCopter(i).getLocation().getLatitude();
            lonSum += API.getCopter(i).getLocation().getLongitude();
        }
        double centerLat = latSum / numUAVs;
        double centerLon = lonSum / numUAVs;
        centerUTM = Location3DGeo.getUTM(centerLat, centerLon, alt);

        if (debugFormacion && numUAV == 0) {
            System.out.println("📌 Verificación visual de formación generada:");
            List<Location3DUTM> polygonPoints = new ArrayList<>();
            for (int i = 0; i < numUAVs; i++) {
                Location3DUTM pos = target.get3DUTMLocation(centerUTM, i);
                System.out.printf("🔘 Pos %d → (%.2f, %.2f)\n", i, pos.x, pos.y);
                drawVectors.drawCircle(pos);
                polygonPoints.add(pos);
            }
            drawVectors.drawPolygon(polygonPoints);
        }

        List<Location3DUTM> currentPositions = new ArrayList<>();
        List<Location3DUTM> targetPositions = new ArrayList<>();
        for (int i = 0; i < numUAVs; i++) {
            currentPositions.add(new Location3DUTM(API.getCopter(i).getLocationUTM(), alt));
            targetPositions.add(target.get3DUTMLocation(centerUTM, i));
        }

        int[] assignment = assignWithMinimalInterference(currentPositions, targetPositions);

        for (int i = 0; i < numUAVs; i++) {
            double dist = currentPositions.get(i).distance3D(targetPositions.get(assignment[i]));
            System.out.printf("📍 Asignación: UAV %d -> Posición %d | Distancia = %.2f\n", i, assignment[i], dist);
        }

        int assignedIndex = assignment[numUAV];
        Location3DUTM targetPos = targetPositions.get(assignedIndex);
        Location3DUTM current = getCopterLocation();
        finalTarget = targetPos;

        System.out.printf("📦 UAV %d - Desde (%.2f, %.2f)\n", numUAV, current.x, current.y);
        System.out.printf("🎯 UAV %d - Waypoint FINAL: (%.2f, %.2f)\n", numUAV, targetPos.x, targetPos.y);

        waypoints.clear();
        waypoints.add(targetPos);
    }

    private Vector getAttractionVector() {
        Location3DUTM current = getCopterLocation();
        Location3DUTM target = waypoints.peek();
        double dist = current.distance3D(target);

        Vector attraction = new Vector(current, target);
        attraction.normalize();

        double speed;
        if (dist > 50) {
            speed = maxspeed;
        } else if (dist > 30) {
            speed = maxspeed * 0.7;
        } else if (dist > 15) {
            speed = maxspeed * 0.5;
        } else if (dist > 7) {
            speed = maxspeed * 0.3;
        } else {
            speed = maxspeed * 0.15;
        }

        if (dist < 1.5) {
            System.out.printf("🛑 UAV %d sin atracción (muy cerca del objetivo, %.2f m)\n", numUAV, dist);
            return new Vector(0, 0);
        }

        System.out.printf("🧲 UAV %d - Atracción %.2f m/s hacia objetivo (%.2f m)\n", numUAV, speed, dist);

        attraction.scalarProduct(speed);
        return attraction;
    }

    private Vector calculateHybridRepulsion() {
        Vector total = new Vector();
        List<Pair<Location3DUTM, Double>> obstacles = communication.getObstaclesWithHeading();
        Vector headingVec = headingVec();
        double cutoff = 15.0; // escudo protector máximo 15 m
        double mu = magneticSwarmRecSimProperties.dirFactor;
        double dirRatio = magneticSwarmRecSimProperties.dirRatio;

        for (Pair<Location3DUTM, Double> obs : obstacles) {
            Location3DUTM obstacle = obs.getValue0();
            double dist = getCopterLocation().distance3D(obstacle);
            if (dist < cutoff) {
                Vector rep = new Vector(obstacle, getCopterLocation());
                rep.normalize();

                double dot = rep.dot(headingVec);
                double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dot / (rep.magnitude() * headingVec.magnitude()))));
                double directionFactor = Math.max(Math.cos(mu * angle), dirRatio);

                double omniStrength = calculateOmniMagnitude(dist);
                Vector omni = rep.scaledCopy(1.0);
                omni.scalarProduct(omniStrength);

                Vector directional = rep.scaledCopy(directionFactor);
                directional.scalarProduct(omniStrength * 0.7);

                if (communication.isUAVHovering(obstacle)) omni.scalarProduct(2.0);

                total = Vector.add(total, omni);
                total = Vector.add(total, directional);
            }
        }

        if (getCopterLocation().distance3D(getLastTarget()) < 6.0) {
            total.scalarProduct(0.3);
        }

        double maxRepulsion = maxspeed * 0.6;
        if (total.magnitude() > maxRepulsion) {
            total.normalize();
            total.scalarProduct(maxRepulsion);
        }

        return total;
    }

    private double calculateOmniMagnitude(double distance) {
        if (distance >= 15.0) {
            return 0.0;
        } else if (distance >= 10.0) {
            return maxspeed * (1.0 - (distance - 10.0) / 5.0);
        } else if (distance >= 1.0) {
            return maxspeed * 1.0;
        } else {
            return maxspeed * distance;
        }
    }

    private void penalizeAttractionIfNeeded(Vector attraction, Vector repulsion, double dist) {
        double repulsionMag = repulsion.magnitude();
        if (repulsionMag > 0.2 * maxspeed && dist > 5.0) {
            double penaltyFactor = Math.max(0.5, 1.0 - repulsionMag / maxspeed);
            attraction.scalarProduct(penaltyFactor);
            System.out.printf("🐢 UAV %d reduce velocidad por repulsión: factor=%.2f | Repulsión=%.2f m/s\n",
                    numUAV, penaltyFactor, repulsionMag);
        }
    }

    private void sendVelocityCommand(Vector velocity) {
        if (velocity.magnitude() > maxspeed) {
            velocity.normalize();
            velocity.scalarProduct(maxspeed);
        }
        moveUAV(velocity);
    }

    private void moveUAV(Vector v) {
        copter.moveTo(v.y, v.x, 0);
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), copter.getAltitude());
    }

    private boolean isUAVStable() {
        double[] speed = copter.getSpeedComponents();
        double vx = speed[0];
        double vy = speed[1];
        double horizontalSpeed = Math.sqrt(vx * vx + vy * vy);
        return horizontalSpeed < 0.5;
    }

    private Vector headingVec() {
        return new Vector(Math.cos(getHeading()), Math.sin(getHeading()));
    }

    private double getHeading() {
        return Math.toRadians(copter.getHeading());
    }

    private Location3DUTM getLastTarget() {
        return waypoints.isEmpty() ? getCopterLocation() : waypoints.peek();
    }

    private int[] assignWithMinimalInterference(List<Location3DUTM> current, List<Location3DUTM> target) {
        int n = current.size();
        double[][] costMatrix = new double[n][n];
        double angleWeight = 30.0; // Peso para penalizar ángulo en la asignación

        for (int i = 0; i < n; i++) {
            Location3DUTM from = current.get(i);
            for (int j = 0; j < n; j++) {
                Location3DUTM to = target.get(j);
                double dist = from.distance3D(to);

                double dx = to.x - from.x;
                double dy = to.y - from.y;

                double heading = API.getCopter(i).getHeading();
                double headingRad = Math.toRadians(heading);
                double dirX = Math.cos(headingRad);
                double dirY = Math.sin(headingRad);

                double dot = dx * dirX + dy * dirY;
                double magA = Math.hypot(dx, dy);
                double magB = Math.hypot(dirX, dirY);
                double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dot / (magA * magB))));

                costMatrix[i][j] = dist + angleWeight * angle;
            }
        }

        HungarianAlgorithm hungarian = new HungarianAlgorithm(costMatrix);
        return hungarian.execute();
    }
}
