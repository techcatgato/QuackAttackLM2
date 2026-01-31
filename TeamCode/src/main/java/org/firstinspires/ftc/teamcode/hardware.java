package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class hardware {
    //motors
    public MotorEx leftBack;
    public MotorEx leftFront;
    public MotorEx rightBack;
    public MotorEx rightFront;
    public MotorEx turret;
    public MotorEx intake;
    public MotorEx shooterLeft;
    public MotorEx shooterRight;
    //servos
    public ServoEx hoodLeft;
    public ServoEx hoodRight;
    //TODO might remove
    public CRServoEx transfer;
    //localizers
    public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight;
    public void init(HardwareMap hwMap) {
        leftBack = new MotorEx(hwMap, "leftBack");
        leftFront = new MotorEx(hwMap, "leftFront");
        rightBack = new MotorEx(hwMap, "rightBack");
        rightFront = new MotorEx(hwMap, "rightFront");
        turret = new MotorEx(hwMap, "turret");
        intake = new MotorEx(hwMap, "intake");
        shooterLeft = new MotorEx(hwMap, "shooterLeft");
        shooterRight = new MotorEx(hwMap, "shooterRight");

        hoodLeft = new ServoEx(hwMap, "hoodLeft");
        hoodRight = new ServoEx(hwMap, "hoodRight");
        //TODO
        transfer = new CRServoEx(hwMap, "transfer");

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        
    }

}
