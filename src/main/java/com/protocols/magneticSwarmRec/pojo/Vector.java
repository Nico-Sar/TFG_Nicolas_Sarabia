package com.protocols.magneticSwarmRec.pojo;

import es.upv.grc.mapper.Location3DUTM;

public class Vector {

    public double x, y, z;

    public Vector(Location3DUTM l1, Location3DUTM l2) {
        this.x = l2.x - l1.x;
        this.y = l2.y - l1.y;
        this.z = 0; // Adaptación: solo plano XY
    }

    public Vector(Vector v) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }

    public Vector() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    private Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public String toString() {
        return String.format("{%.2f;%.2f;%.2f}", x, y, z);
    }

    public void add(Vector v) {
        this.x += v.x;
        this.y += v.y;
        // z permanece 0 por diseño
    }

    public static Vector add(Vector v1, Vector v2) {
        if (v1 == null) return v2;
        if (v2 == null) return v1;
        return new Vector(v1.x + v2.x, v1.y + v2.y, 0); // plano XY
    }

    public void subtract(Vector v) {
        this.x -= v.x;
        this.y -= v.y;
        // z permanece 0
    }

    public void scalarProduct(double s) {
        this.x *= s;
        this.y *= s;
        // z permanece 0
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y); // plano XY
    }

    public void normalize() {
        double m = magnitude();
        if (m > 0) {
            this.x /= m;
            this.y /= m;
            // z permanece 0
        }
    }
}
