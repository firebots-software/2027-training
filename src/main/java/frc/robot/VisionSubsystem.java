package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionSubsystem {
    private Optional<EstimatedRobotPose> visionEst;
    PhotonCamera camera;
    private static final AprilTagFieldLayout kTagLayout;
    private static final Transform3d kRobotToCam;
    private PhotonPoseEstimator photonEstimator;
    public VisionSubsystem (){
        camera = new PhotonCamera("photonvision2");
        kTagLayout = 
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        kRobotToCam = // update later
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

    public Optional<EstimatedRobotPose> getVisionEst() { 
        EstimatedRobotPose estimate = visionEst.get();
        Pose2d finalPose2d = estimate.estimatedPose.toPose2d();
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


