package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.baseAlliance;

public class shooter {
    public static double kS = 0.08, kV = 0.00039, kP = 0.01;
    public double targetVelocity = 0;
    public double currentVelocity = 0;
    public double powerCommand;
    public Motor shooterLeft;
    public Motor shooterRight;
    public shooter(Motor shooterLeft, Motor shooterRight) {
        this.shooterLeft = shooterLeft;
        this.shooterRight = shooterRight;
        shooterRight.setInverted(true);

    }

    public void calculate(Pose2D pose, baseAlliance baseAlliance) {
        currentVelocity = (shooterLeft.getCorrectedVelocity() + shooterRight.getCorrectedVelocity()) / 2;
        //TODO calculate target correctly
        powerCommand = (kV * targetVelocity) + (kP * (targetVelocity - currentVelocity)) + kS;
    }
    public void updateMotors() {
        shooterLeft.set(powerCommand);
        shooterRight.set(powerCommand);
    }
}
