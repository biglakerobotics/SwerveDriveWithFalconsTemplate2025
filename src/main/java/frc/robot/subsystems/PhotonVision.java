package frc.robot.subsystems;

import java.io.PipedInputStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


public class PhotonVision {
    
    private final PhotonCamera photonCamera = new PhotonCamera("stupidcamera");
    
    public PhotonVision(PhotonCamera photonCamera) {
        var pipelineResult = photonCamera.getLatestResult();
        if (pipelineResult.hasTargets()) {
            var target = pipelineResult.getBestTarget();
            var targetYaw = target.getYaw();
            var targetPitch = target.getPitch();
            var camToTarget = target.getBestCameraToTarget();
        }
    }
}
