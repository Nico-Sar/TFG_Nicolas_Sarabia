package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.copter.Copter;
import com.api.copter.TakeOff;
import com.api.copter.TakeOffListener;
import com.api.pojo.location.Waypoint;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.Vector;
import com.uavController.UAVParam;
import es.upv.grc.mapper.*;
import net.objecthunter.exp4j.Expression;
import net.objecthunter.exp4j.ExpressionBuilder;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class magneticSwarmAvoidance extends Thread {

    Queue<Location3DUTM> waypoints;
    Copter copter;
    int numUAV;
    int numUAVs;
    Communication communication;
    double minDistance = Double.MAX_VALUE;
    double maxspeed;

    public magneticSwarmAvoidance(int numUAV){
        this.numUAV = numUAV;
        this.numUAVs = API.getArduSim().getNumUAVs();
        this.copter = API.getCopter(numUAV);
        setWaypoints(numUAV);
        communication = new Communication(numUAV);
        maxspeed = copter.getPlannedSpeed();
    }

    private void setWaypoints(int numUAV) {
        waypoints = new LinkedList<>();
        List<Waypoint>[] missions = copter.getMissionHelper().getMissionsLoaded();
        for(int i = 1; i<missions[numUAV].size(); i++){
            waypoints.add(new Location3DUTM(missions[numUAV].get(i).getUTM(), magneticSwarmRecSimProperties.altitude));
        }
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), copter.getAltitude());
    }

    @Override
    public void run(){
        takeoff();
        communication.start();
        long start = System.currentTimeMillis();
        while (waypoints.size() > 0) {
            while (!waypointReached()) {
                Vector attraction = getAttractionVector();
                Vector totalRepulsion = calculateRepulsion();
                Vector resulting = Vector.add(attraction,totalRepulsion);
                resulting = reduceToMaxSpeed(resulting);
                moveUAV(resulting);
                logMinDistance();
                API.getArduSim().sleep(200);
            }
            waypoints.poll();
        }
        long protocolTime = System.currentTimeMillis() - start;
        communication.stopCommunication();
        land();
        saveData(protocolTime);
    }

    private Vector calculateRepulsion() {
        Vector totalRepulsion = new Vector();
        for(Location3DUTM obstacle:communication.getObstacles()) {
            Vector repulsion = getRepulsionVector(obstacle);
            totalRepulsion = Vector.add(totalRepulsion, repulsion);
        }
        return totalRepulsion;
    }

    private void takeoff() {
        TakeOff takeOff = copter.takeOff(magneticSwarmRecSimProperties.altitude, new TakeOffListener() {
            @Override
            public void onCompleteActionPerformed() {}
            @Override
            public void onFailure() {}
        });
        takeOff.start();
        try {
            takeOff.join();
        } catch (InterruptedException e1) {}
        waypoints.poll();
    }

    private boolean waypointReached(){
        return getCopterLocation().distance3D(waypoints.peek()) < 5;
    }

    private Vector getAttractionVector() {
        Vector attraction = new Vector(getCopterLocation(), waypoints.peek());
        attraction.normalize();
        if(getCopterLocation().distance3D(waypoints.peek()) > 50) {
            attraction.scalarProduct(maxspeed);
        }else{
            attraction.scalarProduct(maxspeed/2);
        }
        return attraction;
    }

    private Vector getRepulsionVector(Location3DUTM obstacle) {
        if(obstacle != null) {
            Location3DUTM uav = getCopterLocation();
            Vector repulsionDirection = new Vector(obstacle, uav);
            return changeMagnitude(repulsionDirection, obstacle.distance3D(uav));
        }
        return null;
    }

    private Vector changeMagnitude(Vector repulsion, double distance) {
        repulsion.z = 0;
        repulsion.normalize();
        applyMagnitudeFunction(repulsion, distance);
        return repulsion;
    }

    private void applyMagnitudeFunction(Vector repulsion, double distance) {
        distance = Math.max(magneticSwarmRecSimProperties.frd,distance);
        Expression expression = new ExpressionBuilder(magneticSwarmRecSimProperties.repulsionMagnitude)
                .variables("x","frd","a")
                .build()
                .setVariable("x", distance)
                .setVariable("frd", magneticSwarmRecSimProperties.frd)
                .setVariable("a", magneticSwarmRecSimProperties.a);
        double scalar = expression.evaluate();
        if(scalar < 0){
            scalar = 0;
        }
        repulsion.scalarProduct(scalar);
    }

    private Vector reduceToMaxSpeed(Vector v_) {
        Vector v = new Vector(v_);
        if(v.magnitude() > maxspeed){
            v.normalize();
            v.scalarProduct(maxspeed);
        }
        return v;
    }

    private void moveUAV(Vector resulting) {
        copter.moveTo(resulting.y, resulting.x, 0); // Mantener en plano XY (sin cambio en Z)
    }

    private void logMinDistance() {
        for(int i=0;i<numUAVs;i++) {
            if(numUAV == i){continue;}
            double distance = UAVParam.distances[numUAV][i].get();
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
    }

    private void saveData(long protocolTime) {
        String protocol = magneticSwarmRecSimProperties.missionFile.get(0).toString();
        protocol = protocol.substring(protocol.lastIndexOf("/") + 1);
        protocol = protocol.substring(0, protocol.length() - 4);
        String repulsion = createStringRepulsionFunction();
        int battery = copter.getBattery();
        try {
            File f = new File(protocol);
            boolean includeHeader = !f.exists();
            BufferedWriter writer = new BufferedWriter(new FileWriter(f,true));
            if(includeHeader){
                writer.append("numUAV,numUAVs,repulsion,minDistance(m),protocolTime(ms),battery(%),beaconingTime(ms)\n");
            }

            writer.append(String.valueOf(numUAV))
                    .append(",")
                    .append(String.valueOf(API.getArduSim().getNumUAVs()))
                    .append(",")
                    .append(repulsion)
                    .append(",")
                    .append(String.valueOf(minDistance))
                    .append(",")
                    .append(String.valueOf(protocolTime))
                    .append(",")
                    .append(String.valueOf(battery))
                    .append(",")
                    .append(String.valueOf(magneticSwarmRecSimProperties.beaconingTime))
                    .append("\n");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private String createStringRepulsionFunction() {
        String s = magneticSwarmRecSimProperties.repulsionMagnitude;
        String a = String.valueOf(magneticSwarmRecSimProperties.a);
        String frd = String.valueOf(magneticSwarmRecSimProperties.frd);
        s = s.replace("a",a);
        s = s.replace("frd",frd);
        return s;
    }

    private void land() {
        copter.land();
    }
}
