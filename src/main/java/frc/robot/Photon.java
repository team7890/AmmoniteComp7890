// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {

  static PhotonCamera objCamera = new PhotonCamera("dragon");
    static double dYaw;
    static double dPitch;
    static double dArea;
    double dSkew;

    static double dDistance;
    
    /** Creates a new Photon. */
    public Photon() {
      // var result = objCamera.getLatestResult();
      // boolean bHasTarget = result.hasTargets();
      // List<PhotonTrackedTarget> targetsList = result.getTargets();
      // PhotonTrackedTarget target = result.getBestTarget();
  
      // // Get information from target.
      // dYaw = target.getYaw();
      // double dPitch = target.getPitch();
      // double dArea = target.getArea();
      // double dSkew = target.getSkew();
      
      // int targetID = target.getFiducialId();
      // double poseAmbiguity = target.getPoseAmbiguity();
      // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
  
      
    
    }
  
    public static double PhotonYaw (){
      var result = objCamera.getLatestResult();
      boolean bHasTarget = result.hasTargets();
      
      List<PhotonTrackedTarget> targetsList = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();
     
      // Get information from target.
      if (bHasTarget) {
        dYaw = target.getYaw();
      }
      else {dYaw = 7890.0;}
      
      // double dPitch = target.getPitch();
      // double dArea = target.getArea();
      // double dSkew = target.getSkew();
      
      // int targetID = target.getFiducialId();
      // double poseAmbiguity = target.getPoseAmbiguity();
      // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
      return dYaw;
    }
  
    public static double PhotonPitch (){
      var result = objCamera.getLatestResult();
    boolean bHasTarget = result.hasTargets();
    List<PhotonTrackedTarget> targetsList = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    // Get information from target.
    // dYaw = target.getYaw();
    if (bHasTarget) {
      dPitch = target.getPitch();
    }
    else {dPitch = 7890.0;}
    // double dArea = target.getArea();
    // double dSkew = target.getSkew();
    
    // int targetID = target.getFiducialId();
    // double poseAmbiguity = target.getPoseAmbiguity();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    return dPitch;
  }

  public double PhotonArea (){
    var result = objCamera.getLatestResult();
    boolean bHasTarget = result.hasTargets();
    List<PhotonTrackedTarget> targetsList = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    // Get information from target.
    // dYaw = target.getYaw();
    // double dPitch = target.getPitch();
    if (bHasTarget) {
      dArea = target.getArea();
    }
    else {dArea = 7890.0;}
    // double dSkew = target.getSkew();
    
    // int targetID = target.getFiducialId();
    // double poseAmbiguity = target.getPoseAmbiguity();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    return dArea;
  }

  public double PhotonSkew(){
    var result = objCamera.getLatestResult();
    boolean bHasTarget = result.hasTargets();
    List<PhotonTrackedTarget> targetsList = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    // Get information from target.
    // dYaw = target.getYaw();
    // double dPitch = target.getPitch();
    // double dArea = target.getArea();
    double dSkew = target.getSkew();
    
    // int targetID = target.getFiducialId();
    // double poseAmbiguity = target.getPoseAmbiguity();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    return dSkew;
  }

  public double PhotonDist(){
    var result = objCamera.getLatestResult();
    boolean bHasTarget = result.hasTargets();
    List<PhotonTrackedTarget> targetsList = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    if (bHasTarget) {
      dDistance = bestCameraToTarget.getX();
      return dDistance;
    }
    else return 7890.0;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
