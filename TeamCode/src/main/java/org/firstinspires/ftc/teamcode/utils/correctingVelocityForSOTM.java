package org.firstinspires.ftc.teamcode.utils;

public class correctingVelocityForSOTM {
    public double correctVelocityForSOTM(double baseShooterVelocity, double targetRad, double xVel, double yVel) {
        double kp = 0.3; //tunable
        double correctedYVel = (xVel * Math.sin(targetRad)) + (yVel * Math.cos(targetRad));
        return baseShooterVelocity - (correctedYVel * kp);
    }
}
