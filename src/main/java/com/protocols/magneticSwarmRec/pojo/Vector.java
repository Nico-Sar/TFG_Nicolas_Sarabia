package com.protocols.magneticSwarmRec.pojo;

import es.upv.grc.mapper.Location3DUTM;

public class Vector {

    public double x, y, z;

    // Constructor por defecto
    public Vector() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }
    // Constructor desde heading (ángulo en radianes)
    public Vector(double angle) {
        this.x = Math.cos(angle);
        this.y = Math.sin(angle);
        this.z = 0;
    }

    // Constructor de copia
    public Vector(Vector v) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }
    // Constructor explícito 2D (z = 0)
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
        this.z = 0;
    }


    // Constructor desde dos ubicaciones (de -> hacia)
    public Vector(Location3DUTM from, Location3DUTM to) {
        this.x = to.x - from.x;
        this.y = to.y - from.y;
        this.z = 0; // 2D
    }

    // Constructor explícito
    public Vector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Constructor desde ángulo (heading) en radianes y magnitud
    public static Vector fromAngle(double angle, double magnitude) {
        double x = magnitude * Math.cos(angle);
        double y = magnitude * Math.sin(angle);
        return new Vector(x, y, 0);
    }

    // Producto escalar
    public static double dot(Vector v1, Vector v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }
    public Vector scaledCopy(double factor) {
        return new Vector(this.x * factor, this.y * factor, 0);
    }

    public double dot(Vector other) {
        return x * other.x + y * other.y;
    }

    // Suma en instancia
    public void add(Vector v) {
        this.x += v.x;
        this.y += v.y;
    }

    // Suma estática
    public static Vector add(Vector v1, Vector v2) {
        if (v1 == null) return v2;
        if (v2 == null) return v1;
        return new Vector(v1.x + v2.x, v1.y + v2.y, 0);
    }

    // Escalado
    public void scalarProduct(double factor) {
        this.x *= factor;
        this.y *= factor;
    }

    public static Vector subtract(Vector v1, Vector v2) {
        return new Vector(v1.x - v2.x, v1.y - v2.y, 0);
    }

    public void negate() {
        this.x = -this.x;
        this.y = -this.y;
    }
    public Vector normalized() {
        double mag = this.magnitude();
        if (mag == 0.0) return new Vector(0, 0);
        return new Vector(this.x / mag, this.y / mag);
    }


    public void multiply(double factor) {
        this.scalarProduct(factor);
    }


    // Normalización
    public void normalize() {
        double mag = magnitude();
        if (mag > 0) {
            this.x /= mag;
            this.y /= mag;
        }
    }

    // Magnitud (módulo)
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    // Dirección en radianes desde origen
    public double direction() {
        return Math.atan2(y, x);
    }

    // Para debug o logging
    @Override
    public String toString() {
        return String.format("{%.2f; %.2f}", x, y);
    }
}
