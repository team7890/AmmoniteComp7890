// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

 import java.lang.reflect.Array;
 import java.util.List;

 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonUtils;
 import org.photonvision.targeting.PhotonTrackedTarget;
 import org.photonvision.targeting.TargetCorner;

 import edu.wpi.first.math.geometry.Transform2d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {

  static PhotonCamera objDragon = new PhotonCamera("dragon");
   static PhotonCamera objTrain = new PhotonCamera("train");
     static double dYaw;
    static double dPitch;
    static double dArea;
  
    double dSkew;

     static double dDistance;
    
     /** Creates a new Photon. */
     public Photon() {

    }
  
     public static double PhotonYaw (){
       var resultDragon = objDragon.getLatestResult();
       boolean bHasTargetDragon = resultDragon.hasTargets();
      
      List<PhotonTrackedTarget> targetsList = resultDragon.getTargets();
       PhotonTrackedTarget target = resultDragon.getBestTarget();

       var resultTrain = objTrain.getLatestResult();
      boolean bTrainTarget = resultTrain.hasTargets();

      List<PhotonTrackedTarget> targetsListTrain = resultTrain.getTargets();
      PhotonTrackedTarget targetTrain = resultTrain.getBestTarget();
     
      

      if (bHasTargetDragon & bTrainTarget) {
       dYaw = (target.getYaw() + targetTrain.getYaw()) / 2;
       }
      else {dYaw = 7890.0;}
      
      return dYaw;
    }
  
    public static double PhotonPitch (){
    var resultDragon = objDragon.getLatestResult();
       boolean bHasTargetDragon = resultDragon.hasTargets();
      
       List<PhotonTrackedTarget> targetsList = resultDragon.getTargets();
      PhotonTrackedTarget target = resultDragon.getBestTarget();

       var resultTrain = objTrain.getLatestResult();
      boolean bTrainTarget = resultTrain.hasTargets();
      
      List<PhotonTrackedTarget> targetsListTrain = resultTrain.getTargets();
       PhotonTrackedTarget targetTrain = resultTrain.getBestTarget();
     
       // Get information from target.
       if (bHasTargetDragon & bTrainTarget) {
         dPitch = (target.getPitch() + targetTrain.getPitch()) / 2;
       }
      else {dPitch = 7890.0;}
      
      return dPitch;
  }

   public double PhotonArea (){
     var resultDragon = objDragon.getLatestResult();
    boolean bHasTargetDragon = resultDragon.hasTargets();
      
    List<PhotonTrackedTarget> targetsList = resultDragon.getTargets();
   PhotonTrackedTarget target = resultDragon.getBestTarget();

    var resultTrain = objTrain.getLatestResult();
   boolean bTrainTarget = resultTrain.hasTargets();
      
    List<PhotonTrackedTarget> targetsListTrain = resultTrain.getTargets();
   PhotonTrackedTarget targetTrain = resultTrain.getBestTarget();
     
     // Get information from target.
    if (bHasTargetDragon & bTrainTarget) {
       dArea = (target.getArea() + targetTrain.getArea()) / 2;
     }
    else {dArea = 7890.0;}
      
     return dArea;
  }

  public double PhotonDist(){

//     // === DRAGON === \\
    var resultDragon = objDragon.getLatestResult();
     boolean bDragonTarget = resultDragon.hasTargets();
    List<PhotonTrackedTarget> targetsList = resultDragon.getTargets();
     PhotonTrackedTarget targetDragon = resultDragon.getBestTarget();
    
     int targetDragonID = targetDragon.getFiducialId();
     double poseAmbiguityDragon = targetDragon.getPoseAmbiguity();
    Transform3d bestDragon = targetDragon.getBestCameraToTarget();
     Transform3d altDragon = targetDragon.getAlternateCameraToTarget();

//    // === TRAIN === \\
    var resultTrain = objTrain.getLatestResult();
    boolean bTrainTarget = resultTrain.hasTargets();
     List<PhotonTrackedTarget> targetsListTrain = resultTrain.getTargets();
    PhotonTrackedTarget targetTrain = resultTrain.getBestTarget();

     int targetTrainID = targetTrain.getFiducialId();
     double poseAmbiguityTrain = targetTrain.getPoseAmbiguity();
     Transform3d bestTrain = targetTrain.getBestCameraToTarget();
    Transform3d altTrain = targetTrain.getAlternateCameraToTarget();

     if (bDragonTarget & bTrainTarget) {
       dDistance = (bestDragon.getX() + bestTrain.getX()) / 2;
       return dDistance;
    }
    else return 7890.0;
     }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
