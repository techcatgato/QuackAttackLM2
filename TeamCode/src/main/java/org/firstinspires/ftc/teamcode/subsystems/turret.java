package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class turret {
    public static final double left_corner_blue_basket_x = 182;
    public static final double left_corner_blue_basket_y = 3006.1;
    public static final double right_corner_blue_basket_x = 584.6;
    public static final double right_corner_blue_basket_y = 3561.3;

    public static final double right_corner_red_basket_x = 2985.6;
    public static final double right_corner_red_basket_y = 3561.3;
    public static final double left_corner_red_basket_x = 3388.2;
    public static final double left_corner_red_basket_y = 3006.1;

    public static final double ticksPerFullSpin = 1344;
    public static final double ticksPerRad = ticksPerFullSpin / (2*Math.PI);
    double targetFieldRad;

    double targetTurretRad;
    double targetTurretTicks;
    double targetX;
    double targetY;
    double deltaX;
    double deltaY;
    double deltaRad;
    int currentTicks;
    double currentTurretRad;
    Motor turretMotor;
    public static final double turretPower = 0.5;

    public turret (Motor turretMotor, char color) {
        turretMotor.setRunMode(Motor.RunMode.PositionControl);
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.turretMotor = turretMotor;
        if (color == 'r') {
            targetX = (right_corner_red_basket_x + left_corner_red_basket_x) / 2;
            targetY = (right_corner_red_basket_y + left_corner_red_basket_y)/2;
        } else {
            targetX = (right_corner_blue_basket_x + left_corner_blue_basket_x) / 2;
            targetY = (right_corner_blue_basket_y + left_corner_blue_basket_y) / 2;
        }

    }
    public void track(Pose2D currentPose) {
        deltaX = targetX - currentPose.getX(DistanceUnit.MM);
        deltaY = targetY - currentPose.getY(DistanceUnit.MM);
        targetFieldRad = Math.atan2(deltaY, deltaX);
        targetTurretRad = AngleUnit.normalizeRadians(targetFieldRad - currentPose.getHeading(AngleUnit.RADIANS));
        currentTicks = turretMotor.getCurrentPosition();
        currentTurretRad = AngleUnit.normalizeRadians(currentTicks/ ticksPerRad);
        deltaRad = AngleUnit.normalizeRadians(targetTurretRad - currentTurretRad);
        targetTurretTicks = Math.round(currentTicks + (deltaRad * ticksPerRad));
        turretMotor.setTargetPosition((int) targetTurretTicks); //tell SolverLib controller the target
        turretMotor.set(turretPower); //drive to the target with turretPower power


    }

}
