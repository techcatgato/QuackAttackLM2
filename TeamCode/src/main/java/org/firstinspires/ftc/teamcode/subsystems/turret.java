package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class turret {

    enum turretStates {
        IDLE,
//        HOMING, todo
        AUTOMATIC,
        MANUAL
    }
    public double deadBand = Math.toRadians(0.1);
    public double power = 0;

    double kP = 3.5;

    double kD = 0.06;
    double kS = 0.05;


    public double kV = 0.0; //try 0.02-0.08
    private double prevTargetRad = 0.0;
    private boolean hasPrev = false;
    private long lastNano = 0L;

    public static final double left_corner_blue_basket_x = 182;
    public static final double left_corner_blue_basket_y = 3006.1;
    public static final double right_corner_blue_basket_x = 584.6;
    public static final double right_corner_blue_basket_y = 3561.3;

    public static final double right_corner_red_basket_x = 2985.6;
    public static final double right_corner_red_basket_y = 3561.3;
    public static final double left_corner_red_basket_x = 3388.2;
    public static final double left_corner_red_basket_y = 3006.1;

    public static final double ticksPerFullSpin = 1344;
    public static final double ticksPerRad = ticksPerFullSpin / (2 * Math.PI);
    public static final double radPerTick = (2 * Math.PI) / ticksPerFullSpin;

    public static final double maxTurretDeg = 190.0;
    public static final double maxTurretRad = Math.toRadians(maxTurretDeg);

    public static final int limitMarginTicks = 10;
    public static final int minTurretTicks = (int) Math.round(-maxTurretRad * ticksPerRad) + limitMarginTicks;
    public static final int maxTurretTicks = (int) Math.round(maxTurretRad * ticksPerRad) - limitMarginTicks;

    public double targetTurretRad;
    public double targetTurretTicks;
    public double targetX;
    public double targetY;
    public double deltaX;
    public double deltaY;
    public double deltaRad;
    public int currentTicks;
    public double currentTurretRad;
    DcMotorEx turretMotor;
    double dtSec;
    long nowNano;
    double sinH;
    double cosH;
    double forward;
    double left;
    double targetVelRad = 0.0;
    double dTarget;
    public turret(DcMotorEx turretMotor, char color) {
        this.turretMotor = turretMotor;

        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (color == 'r') {
            targetX = (right_corner_red_basket_x + left_corner_red_basket_x) / 2;
            targetY = (right_corner_red_basket_y + left_corner_red_basket_y) / 2;
        } else {
            targetX = (right_corner_blue_basket_x + left_corner_blue_basket_x) / 2;
            targetY = (right_corner_blue_basket_y + left_corner_blue_basket_y) / 2;
        }


        lastNano = System.nanoTime();
    }

    public void track(Pose2D currentPose) {

        nowNano = System.nanoTime();
        dtSec = (nowNano - lastNano) * 1e-9;
        lastNano = nowNano;



        deltaX = targetX - currentPose.getX(DistanceUnit.MM);
        deltaY = targetY - currentPose.getY(DistanceUnit.MM);

        sinH = Math.sin(currentPose.getHeading(AngleUnit.RADIANS));
        cosH = Math.cos(currentPose.getHeading(AngleUnit.RADIANS));
        forward = (-sinH) * deltaX + (cosH) * deltaY;
        left = (-cosH) * deltaX + (-sinH) * deltaY;

        targetTurretRad = Math.atan2(left, forward);

        //new addition, feedforward, todo check
        targetVelRad = 0.0;
        if (hasPrev) {
            dTarget = AngleUnit.normalizeRadians(targetTurretRad - prevTargetRad);
            targetVelRad = dTarget / dtSec;
        }
        prevTargetRad = targetTurretRad;
        hasPrev = true;

        currentTicks = turretMotor.getCurrentPosition();
        currentTurretRad = AngleUnit.normalizeRadians(currentTicks * radPerTick * -1);

        double delta1 = AngleUnit.normalizeRadians(targetTurretRad - currentTurretRad);
        double delta2 = delta1 - Math.copySign(2.0 * Math.PI, delta1);

        int cand1 = (int) Math.round(currentTicks - (delta1 * ticksPerRad));
        int cand2 = (int) Math.round(currentTicks - (delta2 * ticksPerRad));

        boolean cand1Ok = cand1 >= minTurretTicks && cand1 <= maxTurretTicks;
        boolean cand2Ok = cand2 >= minTurretTicks && cand2 <= maxTurretTicks;

        if (cand1Ok && cand2Ok) {
            if (Math.abs(delta1) <= Math.abs(delta2)) {
                targetTurretTicks = cand1;
            } else {
                targetTurretTicks = cand2;
            }
        } else if (cand1Ok) {
            targetTurretTicks = cand1;
        } else if (cand2Ok) {
            targetTurretTicks = cand2;
        } else {
            targetTurretTicks = clamp(cand1, minTurretTicks, maxTurretTicks);
        }

        deltaRad = (currentTicks - targetTurretTicks) * radPerTick;

        double p = 0, d = 0, s = 0, v = 0;

        if (Math.abs(deltaRad) > deadBand) {
            p = kP * deltaRad;
            d = -kD * turretMotor.getVelocity() * radPerTick;

            if (Math.abs(deltaRad) > Math.toRadians(0.3)) s = kS * Math.signum(deltaRad);
            else s = 0;

            v = kV * targetVelRad; // feedforward (because target is moving)
        }

        power = clamp(p + d + s + v, -1.0, 1.0);
        turretMotor.setPower(-power);
    }
}
