package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "Flywheel Velocity Tuner", group = "Tuning")
public class flywheelVelocityTuner extends LinearOpMode {

    // Which parameter is currently selected
    private enum Param {
        kS, kV, kP, TARGET_VEL
    }

    @Override
    public void runOpMode() {
        MotorEx shooterLeft = new MotorEx(hardwareMap, "shooterLeft");
        MotorEx shooterRight = new MotorEx(hardwareMap, "shooterRight");
        shooterRight.setInverted(true);
        
        shooterLeft.setRunMode(Motor.RunMode.RawPower);
        shooterRight.setRunMode(Motor.RunMode.RawPower);

        // -------- Defaults (edit these) --------
        final double kS_DEFAULT = 0.08;
        final double kV_DEFAULT = 0.00039;
        final double kP_DEFAULT = 0.01;
        final double TARGET_DEFAULT = 0.0;

        final double kS_STEP_DEFAULT = 0.01;
        final double kV_STEP_DEFAULT = 0.00001;
        final double kP_STEP_DEFAULT = 0.001;
        final double TARGET_STEP_DEFAULT = 50.0; // ticks/s or rpm-equivalent, set how you like

        // -------- Tunables --------
        double kS = kS_DEFAULT, kV = kV_DEFAULT, kP = kP_DEFAULT;
        double targetVelocity = TARGET_DEFAULT;

        // Per-parameter step sizes (the “increase amount”)
        double kSStep = kS_STEP_DEFAULT, kVStep = kV_STEP_DEFAULT, kPStep = kP_STEP_DEFAULT;
        double targetStep = TARGET_STEP_DEFAULT;

        Param selected = Param.kS;

        telemetry.addLine("Flywheel tuner ready.");
        telemetry.addLine("Dpad L/R: select   Dpad U/D: +/- step");
        telemetry.addLine("X: step*10   B: step/10   A: zero   Y: reset defaults");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double currentVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity())/2;
            // ----- Selection -----
            if (gamepad2.dpad_right) {
                selected = next(selected);
                sleep(140); // simple debounce
            } else if (gamepad2.dpad_left) {
                selected = prev(selected);
                sleep(140);
            }

            // ----- Value change -----
            if (gamepad2.dpad_up) {
                switch (selected) {
                    case kS:        kS += kSStep; break;
                    case kV:        kV += kVStep; break;
                    case kP:        kP += kPStep; break;
                    case TARGET_VEL:targetVelocity += targetStep; break;
                }
                sleep(90);
            } else if (gamepad2.dpad_down) {
                switch (selected) {
                    case kS:        kS -= kSStep; break;
                    case kV:        kV -= kVStep; break;
                    case kP:        kP -= kPStep; break;
                    case TARGET_VEL:targetVelocity -= targetStep; break;
                }
                sleep(90);
            }

            // ----- Step size change for the selected param -----
            if (gamepad2.x) { // step * 10
                switch (selected) {
                    case kS:        kSStep *= 10.0; break;
                    case kV:        kVStep *= 10.0; break;
                    case kP:        kPStep *= 10.0; break;
                    case TARGET_VEL:targetStep *= 10.0; break;
                }
                sleep(140);
            } else if (gamepad2.b) { // step / 10
                switch (selected) {
                    case kS:        kSStep /= 10.0; break;
                    case kV:        kVStep /= 10.0; break;
                    case kP:        kPStep /= 10.0; break;
                    case TARGET_VEL:targetStep /= 10.0; break;
                }
                sleep(140);
            }

            // ----- Quick actions -----
            if (gamepad2.a) { // zero selected
                switch (selected) {
                    case kS:        kS = 0.0; break;
                    case kV:        kV = 0.0; break;
                    case kP:        kP = 0.0; break;
                    case TARGET_VEL:targetVelocity = 0.0; break;
                }
                sleep(200);
            }

            if (gamepad2.y) { // reset all to defaults
                kS = kS_DEFAULT; kV = kV_DEFAULT; kP = kP_DEFAULT; targetVelocity = TARGET_DEFAULT;
                kSStep = kS_STEP_DEFAULT; kVStep = kV_STEP_DEFAULT; kPStep = kP_STEP_DEFAULT; targetStep = TARGET_STEP_DEFAULT;
                selected = Param.kS;
                sleep(250);
            }

            // OPTIONAL: clamp steps so you don't accidentally get absurd tiny/huge steps
            kSStep = clampAbsMin(kSStep, 1e-6);
            kVStep = clampAbsMin(kVStep, 1e-9);
            kPStep = clampAbsMin(kPStep, 1e-6);
            targetStep = clampAbsMin(targetStep, 1e-3);

            // ----- Telemetry -----
            telemetry.addData("Selected", selected);
            telemetry.addLine("Values:");
            telemetry.addData(mark(selected == Param.kS) + "kS", kS);
            telemetry.addData(mark(selected == Param.kV) + "kV", kV);
            telemetry.addData(mark(selected == Param.kP) + "kP", kP);
            telemetry.addData(mark(selected == Param.TARGET_VEL) + "targetVel", targetVelocity);
            telemetry.addData("Real velocity", currentVelocity);
            telemetry.addData("left", shooterLeft.getVelocity());
            telemetry.addData("right", shooterRight.getVelocity());

            telemetry.addLine("Steps:");
            telemetry.addData("kS step", kSStep);
            telemetry.addData("kV step", kVStep);
            telemetry.addData("kP step", kPStep);
            telemetry.addData("target step", targetStep);

            telemetry.update();

            // TODO: plug into your flywheel controller here:
            double powerCommand = (kV * targetVelocity) + (kP * (targetVelocity - currentVelocity)) + kS;
            shooterLeft.set(powerCommand);
            shooterRight.set(powerCommand);

            idle();
        }
    }

    private static Param next(Param p) {
        int i = (p.ordinal() + 1) % Param.values().length;
        return Param.values()[i];
    }

    private static Param prev(Param p) {
        int i = (p.ordinal() - 1 + Param.values().length) % Param.values().length;
        return Param.values()[i];
    }

    private static String mark(boolean selected) {
        return selected ? ">" : " ";
    }

    private static double clampAbsMin(double v, double absMin) {
        if (Math.abs(v) < absMin) return Math.copySign(absMin, v == 0 ? 1.0 : v);
        return v;
    }
}
