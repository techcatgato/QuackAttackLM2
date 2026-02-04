package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.turret;

@TeleOp
public class turretTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setOffsets(-114.5, -136, DistanceUnit.MM);

        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 225, 225, AngleUnit.RADIANS, 0));
        for (int i = 0; i < 5; i++) pinpoint.update();

        waitForStart();

        Pose2D pose;
        turret turret = new turret(turretMotor, 'r');

        while (!isStopRequested()) {
            pinpoint.update();
            pose = pinpoint.getPosition();
            turret.track(pose);

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double h = pose.getHeading(AngleUnit.RADIANS);

            double rotX = strafe * Math.cos(-h) - forward * Math.sin(-h);
            double rotY = strafe * Math.sin(-h) + forward * Math.cos(-h);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);

            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            telemetry.addData("pose", pose);
            telemetry.addData("target", turret.targetTurretRad);
            telemetry.addData("current", turret.currentTurretRad);
            telemetry.addData("power", turret.power);
            telemetry.addLine("rev 67");
            telemetry.update();
        }
    }
}
