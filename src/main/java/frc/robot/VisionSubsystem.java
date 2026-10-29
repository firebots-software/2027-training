package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSubsystem {
    private Optional<EstimatedRobotPose> visionEst;
    PhotonCamera camera = new PhotonCamera("photonvision2");
    private static final AprilTagFieldLayout kTagLayout = 
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final Transform3d kRobotToCam = // update later
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);


    public Optional<EstimatedRobotPose> getVisionEst() { 
        return visionEst; 
    }
    
    public void calcPose() { 
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
    }

}


