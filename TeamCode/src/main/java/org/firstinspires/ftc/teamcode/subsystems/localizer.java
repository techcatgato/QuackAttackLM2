package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

public class localizer {
    Limelight3A limeight;
    GoBildaPinpointDriver pinpoint;
    Pose2D lastPose;
    Pose2D currentPose;
    double[] velocities;

    List<LLResultTypes.FiducialResult> results;
    public localizer(GoBildaPinpointDriver pinpoint, Limelight3A limelight, Pose2D startPose) {
        this.limeight = limelight;
        this.pinpoint = pinpoint;
        this.lastPose = startPose;
    }
    public void update() {
        pinpoint.update();
        results = limeight.getLatestResult().getFiducialResults();
    }
}
