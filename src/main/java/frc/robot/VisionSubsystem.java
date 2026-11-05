package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class VisionSubsystem extends SubsystemBase {
    private Optional<EstimatedRobotPose> visionEst;
    private PhotonCamera rightCamera;
    private static AprilTagFieldLayout kTagLayout;
    private static Transform3d kRobotToCam;
    private PhotonPoseEstimator photonEstimator;
    private String loggingPath = "Robot/Vision/";

    public VisionSubsystem (){
        visionEst = Optional.empty();
        rightCamera = new PhotonCamera("yes");
        kTagLayout = 
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        kRobotToCam = Constants.RightCamera.transform;
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        DogLog.log("its running", true);
    }

    

    public Pose2d get2dPose() { 
        if (visionEst.isEmpty()) {
        return null;
        }
        DogLog.log(loggingPath + "GotPose", visionEst.get().estimatedPose.toPose2d());
        return visionEst.get().estimatedPose.toPose2d();
    }

    
    @Override
    public void periodic() {
        visionEst = Optional.empty();
        for (var result : rightCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
        }
        DogLog.log(loggingPath + "Pose2D", get2dPose());
    }

}


