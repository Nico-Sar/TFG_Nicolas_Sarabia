package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.copter.Copter;
import com.api.copter.TakeOff;
import com.api.copter.TakeOffListener;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.HungarianAlgorithm;
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

    private static final boolean debugFormacion = true;

    public magneticSwarmAvoidance(int numUAV) {
        this.numUAV = numUAV;
        this.numUAVs = API.getArduSim().getNumUAVs();
        this.copter = API.getCopter(numUAV);
        this.drawVectors = new DrawVectors(copter);
        this.communication = new Communication(numUAV);
        this.maxspeed = copter.getPlannedSpeed();
        this.waypoints = new LinkedList<>();
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
        while (true) {
            switch (state) {
                case MOVING:
                    if (waypoints.isEmpty()) {
                        state = UAVState.HOVERING;
                        break;
                    }
                    Location3DUTM destino = waypoints.peek();
                    double dist = getCopterLocation().distance3D(destino);
                    Vector total = Vector.add(getAttractionVector(), (dist < 3.0) ? new Vector(0, 0) : calculateHybridRepulsion());
                    sendVelocityCommand(total);
                    if (dist < threshold && isUAVStable()) {
                        waypoints.poll();
                        attempts = 0;
                    } else if (dist < 5.0 && attempts > 60) {
                        System.out.printf("⚠️ UAV %d está cerca (%.2f m) pero aún se mueve\n", numUAV, dist);
                    }
                    else attempts++;
                    break;

                case HOVERING:
                    // No enviar más movimiento, mantener posición
                    moveUAV(new Vector(0, 0));
                    communication.setHovering(true);
                    if (communication.allUAVsHovering()) {
                        try {
                            // Esperar unos segundos para asegurar que están estables
                            System.out.printf("🛑 UAV %d en posición final. Esperando para aterrizar...\n", numUAV);
                            API.getArduSim().sleep(3000);
                            landingBarrier.await();
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
                    return;
            }
            API.getArduSim().sleep(100);
        }
    }


    private void takeoff() {
        TakeOff takeOff = copter.takeOff(magneticSwarmRecSimProperties.altitude, new TakeOffListener() {
            public void onCompleteActionPerformed() {}
            public void onFailure() {}
        });
        takeOff.start();
        try { takeOff.join(); barrier.await(); } catch (Exception ignored) {}
    }

    private void switchToFlyingFormation() {
        Formation target = FormationFactory.newFormation(
                Formation.Layout.valueOf(magneticSwarmRecSimProperties.flyingFormation.toUpperCase())
        );
        target.init(numUAVs, magneticSwarmRecSimProperties.flyingDistance);
        double alt = magneticSwarmRecSimProperties.altitude;

        // Calcular centro de la formación
        double latSum = 0.0, lonSum = 0.0;
        for (int i = 0; i < numUAVs; i++) {
            latSum += API.getCopter(i).getLocation().getLatitude();
            lonSum += API.getCopter(i).getLocation().getLongitude();
        }
        double centerLat = latSum / numUAVs;
        double centerLon = lonSum / numUAVs;
        centerUTM = Location3DGeo.getUTM(centerLat, centerLon, alt);

        // Visualización
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

        // Posiciones
        List<Location3DUTM> currentPositions = new ArrayList<>();
        List<Location3DUTM> targetPositions = new ArrayList<>();
        for (int i = 0; i < numUAVs; i++) {
            currentPositions.add(new Location3DUTM(API.getCopter(i).getLocationUTM(), alt));
            targetPositions.add(target.get3DUTMLocation(centerUTM, i));
        }

        int[] assignment = assignClosestToEachTarget(currentPositions, targetPositions);

        for (int i = 0; i < numUAVs; i++) {
            double dist = currentPositions.get(i).distance3D(targetPositions.get(assignment[i]));
            System.out.printf("📍 Asignación: UAV %d -> Posición %d | Distancia = %.2f\n", i, assignment[i], dist);
        }

        // Movimiento
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

        // 🔹 Vector unitario hacia el objetivo
        Vector attraction = new Vector(current, target);
        attraction.normalize();

        // 🔹 Cálculo dinámico de velocidad/atracción
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

        // 🔸 Apagado total de atracción si ya estamos pegados
        if (dist < 1.5) {
            System.out.printf("🛑 UAV %d sin atracción (muy cerca del objetivo, %.2f m)\n", numUAV, dist);
            return new Vector(0, 0);
        }

        // 🔸 Logging para depuración
        System.out.printf("🧲 UAV %d - Atracción %.2f m/s hacia objetivo (%.2f m)\n", numUAV, speed, dist);

        attraction.scalarProduct(speed);
        return attraction;
    }


    private Vector calculateHybridRepulsion() {
        Vector total = new Vector();
        List<Pair<Location3DUTM, Double>> obstacles = communication.getObstaclesWithHeading();
        Vector headingVec = new Vector(Math.cos(getHeading()), Math.sin(getHeading()));
        double cutoff = magneticSwarmRecSimProperties.frd;
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
                Vector omni = rep.scaledCopy(1.0);
                applyMagnitudeFunction(omni, dist);
                if (communication.isUAVHovering(obstacle)) omni.scalarProduct(1.2);
                total = Vector.add(total, rep.scaledCopy(directionFactor));
                total = Vector.add(total, omni);
            }
        }

        double distToTarget = getCopterLocation().distance3D(getLastTarget());
        if (distToTarget < 6.0) {
            total.scalarProduct(0.3);
        }

        if (total.magnitude() > maxspeed * 0.8) {
            total.normalize();
            total.scalarProduct(maxspeed * 0.8);
        }
        return total;
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
        double[] speed = copter.getSpeedComponents();  // [vx, vy, vz]
        double vx = speed[0];
        double vy = speed[1];
        double horizontalSpeed = Math.sqrt(vx * vx + vy * vy);
        return horizontalSpeed < 0.5;  // se considera estable si casi no se mueve en plano
    }


    private double getHeading() {
        return Math.toRadians(copter.getHeading());
    }

    private Location3DUTM getLastTarget() {
        return waypoints.isEmpty() ? getCopterLocation() : waypoints.peek();
    }

    private void applyMagnitudeFunction(Vector v, double distance) {
        double strength = Math.max(1.0 - (distance / magneticSwarmRecSimProperties.frd), 0.0);
        v.scalarProduct(strength);
    }

    private int[] assignClosestToEachTarget(List<Location3DUTM> current, List<Location3DUTM> target) {
        int n = current.size();
        int[] assignment = new int[n];
        boolean[] usedUAV = new boolean[n];

        for (int j = 0; j < n; j++) {
            double minDist = Double.MAX_VALUE;
            int bestUAV = -1;

            for (int i = 0; i < n; i++) {
                if (!usedUAV[i]) {
                    double dist = current.get(i).distance3D(target.get(j));
                    if (dist < minDist) {
                        minDist = dist;
                        bestUAV = i;
                    }
                }
            }

            assignment[bestUAV] = j;
            usedUAV[bestUAV] = true;
        }

        return assignment;
    }
}
