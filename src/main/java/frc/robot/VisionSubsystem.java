package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera rightCamera;
    private PhotonPipelineResult result;

    List<PhotonTrackedTarget> targets = result.getTargets();
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator photonEstimator;
    private final Transform3d robotToCam;
    private Optional<EstimatedRobotPose> visionEstimate;
    private Pose2d pose;
    
    private final String loggingPath;
    private final String cameraTitle;


    public VisionSubsystem(){
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        photonEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
        rightCamera = new PhotonCamera("camera");
        cameraTitle = "rightCam"; //cameraID.getLoggingName();
        loggingPath = "Subsystems/Vision/" + cameraTitle;
    }

    public void visionMeasurement(List<PhotonPipelineResult> results){
        result = results.get(results.size() - 1);

        visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
    }
    
    public void calculatePose(){
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
        for(var result : rightCamera.getAllUnreadResults()){
            visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
            if(visionEstimate.isEmpty()){
                visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            
        }
    }

    public void calculateFilteredPose() {
        if (visionEstimate.isEmpty()) return;
        EstimatedRobotPose estimatedPose = visionEstimate.get();
        pose = estimatedPose.estimatedPose.toPose2d();
        DogLog.log(loggingPath + "/MeasuredPose", pose);
    }

    @Override
    public void periodic(){
        visionMeasurement(rightCamera.getAllUnreadResults());
        calculatePose();
        calculateFilteredPose();
    }
}
