// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot;

 import java.lang.reflect.Array;
 import java.util.List;

 import edu.wpi.first.units.Unit;

 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonUtils;
 import org.photonvision.targeting.PhotonTrackedTarget;
 import org.photonvision.targeting.TargetCorner;

 import edu.wpi.first.math.geometry.Transform2d;
 import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {

  static PhotonCamera objDragon = new PhotonCamera("Dragon");
   static PhotonCamera objTrain = new PhotonCamera("Train");
     static double dYaw;
    static double dPitch;
    static double dArea;
    private double targetYaw;
    private double dAvgTargetRange, dTrainAvgTargetRange, dDragonAvgTargetRange, targetRange;
    private double dAvgTargetYaw, dDragonAvgTargetYaw, dTrainAvgTargetYaw;
    private double dDragonTargetCount, dTrainTargetCount;
    private double dDragonTargetCount2, dTrainTargetCount2;
    private int iTargetLostCounter, iTargetLostCounter2;
    private boolean bDragonHasTarget, bTrainHasTarget;
    private boolean bDragonHasTarget2, bTrainHasTarget2;
    private double dFilteredAvgTargetRange;
    private int iFidID;

  
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

    SmartDashboard.putNumber("Target Yaw", targetYaw);
    //SmartDashboard.putNumber("Target Avg Yaw", phoYawTest());
    SmartDashboard.putNumber("Target Range", targetRange);
    SmartDashboard.putNumber("Target Avg Range", phoRangeTest());
    //SmartDashboard.putNumber("Target Aim Prop", photonAimProportional());
    SmartDashboard.putNumber("Target Lost Counter", iTargetLostCounter);
    SmartDashboard.putNumber("Target Lost Counter2", iTargetLostCounter2);
    SmartDashboard.putBoolean("Dragon Has Target", bDragonHasTarget);
    SmartDashboard.putBoolean("Train Has Target", bTrainHasTarget);
    SmartDashboard.putBoolean("Dragon Has Target2", bDragonHasTarget2);
    SmartDashboard.putBoolean("Train Has Target2", bTrainHasTarget2);

  }

  public double phoYawTest() { 
    System.out.println("------------------------ pho Yaw Start -------------------------");
    // targetYaw = 0.0;
    // targetRange = 0.0;
    bDragonHasTarget = false;
    bTrainHasTarget = false;

    var resultsDragon = objDragon.getAllUnreadResults();
    var resultsTrain = objTrain.getAllUnreadResults();
    //int[] tags = {26, 28};
    if (!resultsDragon.isEmpty()) {
      System.out.println("pho Yaw Drag not empty");
      // Camera processed a new frame since last
      // Get the last one in the list.
      var resultDragon = resultsDragon.get(resultsDragon.size() - 1);
      if (resultDragon.hasTargets()) {
        System.out.println("pho Yaw Drag has Target");
        // At least one AprilTag was seen by the camera
        dDragonAvgTargetYaw = 0.0;
        dDragonTargetCount = 0.0;
  
        for (var target : resultDragon.getTargets()) {
          iFidID = target.getFiducialId();
          switch (iFidID) {
            case 2, 5, 10, 21, 24, 25, 26, 27, 18:
            // Found Tag 7, record its information
            bDragonHasTarget = true;
            targetYaw = target.getYaw();
            dDragonAvgTargetYaw = dDragonAvgTargetYaw + targetYaw;
            dDragonTargetCount = dDragonTargetCount + 1.0;

            default:
            System.out.println("pho Yaw Drag ID");
          }
        }
      }
    }

    if (!resultsTrain.isEmpty()) {
      System.out.println("pho Yaw not Train empty");
      var resultTrain = resultsTrain.get(resultsTrain.size() - 1);
      if (resultTrain.hasTargets()) {
        System.out.println("pho Yaw Train has target");
        dTrainAvgTargetYaw = 0.0;
        dTrainTargetCount = 0.0;
        // At least one AprilTag was seen by the camera
        for (var target : resultTrain.getTargets()) {
          iFidID = target.getFiducialId();
          switch (iFidID) {
            case 2, 5, 10, 21, 24, 25, 26, 27, 18: // Found Tag 7, record its information
            bTrainHasTarget = true;
            targetYaw = target.getYaw();
            dTrainAvgTargetYaw = dTrainAvgTargetYaw + targetYaw;
            dTrainTargetCount = dTrainTargetCount + 1.0;

            default:
            System.out.println("pho Yaw Train ID");
          }
        }
      }
    }

    // if (bDragonHasTarget || bTrainHasTarget) {   // commented out because bDragonHasTarget is going true when only seeing 28 which should not happen !!!!!!!!!!!!!!!!!!
    if (bTrainHasTarget) {
      if (dDragonTargetCount + dTrainTargetCount > 0.5) {
        dAvgTargetYaw = (dDragonAvgTargetYaw + dTrainAvgTargetYaw) / (dDragonTargetCount + dTrainTargetCount);
        iTargetLostCounter = 0;
      }
    }
    else {
      if (iTargetLostCounter <= 15) {
        iTargetLostCounter = iTargetLostCounter + 1;
      }
      else {
        dAvgTargetYaw = 0.0;
      }
    }

    return dAvgTargetYaw;

  }


public double phoRangeTest() { 
    // targetYaw = 0.0;
    System.out.println("--------------- pho Range start -------------------");
    // targetRange = 0.0;
    bDragonHasTarget2 = false;
    bTrainHasTarget2 = false;
    var resultsDragon = objDragon.getAllUnreadResults();
    var resultsTrain = objTrain.getAllUnreadResults();
    //int[] tags = {26, 28};
    if (!resultsDragon.isEmpty()) {
      System.out.println("pho Range Drag not empty");
      // Camera processed a new frame since last
      // Get the last one in the list.
      var resultDragon = resultsDragon.get(resultsDragon.size() - 1);
      if (resultDragon.hasTargets()) {
        System.out.println("pho Range Drag has target");
        // At least one AprilTag was seen by the camera
        dDragonTargetCount2 = 0.0;
        dDragonAvgTargetRange = 0.0;  // Added for averaging range 
        for (var target : resultDragon.getTargets()) {
          iFidID = target.getFiducialId();
          switch (iFidID) {
            case 2, 5, 10, 21, 24, 25, 26, 27, 18:

           

            // Found Tag 7, record its information
            bDragonHasTarget2 = true;
            dDragonTargetCount2 = dDragonTargetCount2 + 1.0;
            targetRange = target.getArea();
            // targetRange = target.getBestCameraToTarget().getTranslation().getNorm();
            // targetRange =
            //   PhotonUtils.calculateDistanceToTargetMeters(
            //   0.5, // Measured with a tape measure, or in CAD.
            //   1.435, // From 2024 game manual for ID 7
            //   Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
            //   Units.degreesToRadians(target.getPitch()));

            dDragonAvgTargetRange = dDragonAvgTargetRange + targetRange; 

            default:
            System.out.println("pho Range Drag ID");
          }
        }
      }
    }

    if (!resultsTrain.isEmpty()) {
      System.out.println("pho Range Train not empty");
      var resultTrain = resultsTrain.get(resultsTrain.size() - 1);
      if (resultTrain.hasTargets()) {
        System.out.println("pho Range Train has Target");
        dTrainTargetCount2 = 0.0;
        dTrainAvgTargetRange = 0.0;
        // At least one AprilTag was seen by the camera
        for (var target : resultTrain.getTargets()) {
          iFidID = target.getFiducialId();
          switch (iFidID) {
            case 2, 5, 10, 21, 24, 25, 26, 27, 18:
            // Found Tag 7, record its information
            bTrainHasTarget2 = true;
            dTrainTargetCount2 = dTrainTargetCount2 + 1.0;
            targetRange = target.getArea();
            // targetRange = target.getBestCameraToTarget().getTranslation().getNorm();
            // targetRange =
            //   PhotonUtils.calculateDistanceToTargetMeters(
            //   0.5, // Measured with a tape measure, or in CAD.
            //   1.435, // From 2024 game manual for ID 7
            //   Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
            //   Units.degreesToRadians(target.getPitch()));

            dTrainAvgTargetRange = dTrainAvgTargetRange + targetRange;

            default:
            System.out.println("pho Range ID");
          }
        }
      }
    }

    // if (bDragonHasTarget || bTrainHasTarget) {   // commented out because bDragonHasTarget is going true when only seeing 28 which should not happen !!!!!!!!!!!!!!!!!!
    if (bTrainHasTarget2) {
      if (dDragonTargetCount2 + dTrainTargetCount2 > 0.5) { 
        dAvgTargetRange = (dDragonAvgTargetRange + dTrainAvgTargetRange) / (dDragonTargetCount2 + dTrainTargetCount2); // This should average the range from both cameras 
        dFilteredAvgTargetRange = 0.3 * dAvgTargetRange + 0.7 * dFilteredAvgTargetRange;
        iTargetLostCounter2 = 0;
      }
    }
    else {
      if (iTargetLostCounter2 <= 15) {
        iTargetLostCounter2 = iTargetLostCounter2 + 1;
      }
      else {
        dAvgTargetRange = 0.0;
        dFilteredAvgTargetRange = 0.0;
      }
    }

    return Math.max(50.0 - (100.0 * dFilteredAvgTargetRange), 0.0);

  }



  // copied from Photon_Lock
  public double photonAimProportional(){
    double kp = 0.02;
    double dragonRotVel;
    
    // if (Photon.PhotonYaw() > 7888.0) {                           // remove all but else of this if because trying phoYawTest
    //   dragonRotVel = 0.0;
    // }
    // else {
      // dragonRotVel = Photon.PhotonYaw() * kp;
      dragonRotVel = phoYawTest() * kp;   // change to phoYawTest
    // }

    // dragonRotVel *= dMaxAngularRate;   // delete because Photon_Lock has MaxAngRate which is not known here

    dragonRotVel *= -1.0;

    return dragonRotVel;
  }
}
