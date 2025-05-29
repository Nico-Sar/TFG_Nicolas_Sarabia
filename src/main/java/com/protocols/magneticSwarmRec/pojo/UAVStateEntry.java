package com.protocols.magneticSwarmRec.pojo;

import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;

public class UAVStateEntry {

    public long timestamp;
    public Pair<Location3DUTM, Double> positionAndHeading;
    public String state;

    public UAVStateEntry(long timestamp, Pair<Location3DUTM, Double> positionAndHeading, String state) {
        this.timestamp = timestamp;
        this.positionAndHeading = positionAndHeading;
        this.state = state;
    }
}
