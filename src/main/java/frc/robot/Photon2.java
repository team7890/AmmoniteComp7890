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

public class Photon2 extends SubsystemBase {

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
  public Photon2() {

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    phoYawRangeTest();

    SmartDashboard.putNumber("Target Avg Yaw", dAvgTargetYaw);
    SmartDashboard.putNumber("Target Avg Range", dFilteredAvgTargetRange);
    SmartDashboard.putNumber("Target Lost Counter", iTargetLostCounter);
    SmartDashboard.putBoolean("Dragon Has Target", bDragonHasTarget);
    SmartDashboard.putBoolean("Train Has Target", bTrainHasTarget);
    //SmartDashboard.putNumber("Target Aim Prop", photonAimProportional());

 
  }

  public void phoYawRangeTest() {
    // System.out.println("------------------------ pho YawRange Start -------------------------");

    var resultsDragon = objDragon.getAllUnreadResults();
    var resultsTrain = objTrain.getAllUnreadResults();

    if (!resultsDragon.isEmpty()) {
      bDragonHasTarget = false;
      bTrainHasTarget = false;
      // System.out.println("pho YawRange Drag not empty");
      // Camera processed a new frame since last
      // Get the last one in the list.
      var resultDragon = resultsDragon.get(resultsDragon.size() - 1);
      if (resultDragon.hasTargets()) {
       // System.out.println("pho YawRange Drag has Target");
        // At least one AprilTag was seen by the camera
        dDragonAvgTargetYaw = 0.0;
        dDragonAvgTargetRange = 0.0;
        dDragonTargetCount = 0.0;
  
        for (var target : resultDragon.getTargets()) {
          iFidID = target.getFiducialId();
          switch (iFidID) {
            case 2, 5, 10, 21, 24, 25, 26, 27, 18:
              //System.out.println("pho YawRange Drag ID in list " + iFidID);
              bDragonHasTarget = true;
              targetYaw = target.getYaw();
              targetRange = Math.max(50.0 - (100.0 * target.getArea()), 0.0);
              dDragonAvgTargetYaw = dDragonAvgTargetYaw + targetYaw;
              dDragonAvgTargetRange = dDragonAvgTargetRange + targetRange; 
              dDragonTargetCount = dDragonTargetCount + 1.0;
              break;
            default:
              //System.out.println("pho YawRange Drag ID not in list " + iFidID);
          }
        }
      }
    }

    if (!resultsTrain.isEmpty()) {
      //System.out.println("pho YawRange not Train empty");
      var resultTrain = resultsTrain.get(resultsTrain.size() - 1);
      if (resultTrain.hasTargets()) {
       // System.out.println("pho YawRange Train has target");
        dTrainAvgTargetYaw = 0.0;
        dTrainTargetCount = 0.0;
        // At least one AprilTag was seen by the camera
        for (var target : resultTrain.getTargets()) {
          iFidID = target.getFiducialId();
          switch (iFidID) {
            case 2, 5, 10, 21, 24, 25, 26, 27, 18:
              //System.out.println("pho Yaw Train ID in list " + iFidID);
              bTrainHasTarget = true;
              targetYaw = target.getYaw();
              targetRange = Math.max(50.0 - (100.0 * target.getArea()), 0.0);
              dTrainAvgTargetYaw = dTrainAvgTargetYaw + targetYaw;
              dDragonAvgTargetRange = dDragonAvgTargetRange + targetRange; 
              dTrainTargetCount = dTrainTargetCount + 1.0;
              break;
            default:
              //System.out.println("pho Yaw Train ID not in list " + iFidID);
          }
        }
      }
    }

    // if (bDragonHasTarget || bTrainHasTarget) {   // commented out because bDragonHasTarget is going true when only seeing 28 which should not happen !!!!!!!!!!!!!!!!!!
    if (bTrainHasTarget) {
      if (dDragonTargetCount + dTrainTargetCount > 0.5) {
        dAvgTargetYaw = (dDragonAvgTargetYaw + dTrainAvgTargetYaw) / (dDragonTargetCount + dTrainTargetCount);
        dAvgTargetRange = (dDragonAvgTargetRange + dTrainAvgTargetRange) / (dDragonTargetCount + dTrainTargetCount); // This should average the range from both cameras 
        dFilteredAvgTargetRange = 0.3 * dAvgTargetRange + 0.7 * dFilteredAvgTargetRange;
        iTargetLostCounter = 0;
      }
    }
    else {
      if (iTargetLostCounter <= 8) {
        iTargetLostCounter = iTargetLostCounter + 1;
      }
      else {
        dAvgTargetYaw = 0.0;
        dAvgTargetRange = 0.0;
        dFilteredAvgTargetRange = 0.0;
      }
    }

  }



  // copied from Photon_Lock
  public double photonAimProportional(){
    double kp = 0.018;
    double dragonRotVel;
    
    // if (Photon.PhotonYaw() > 7888.0) {                           // remove all but else of this if because trying phoYawTest
    //   dragonRotVel = 0.0;
    // }
    // else {
      // dragonRotVel = Photon.PhotonYaw() * kp;
      dragonRotVel = dAvgTargetYaw * kp;   // change to phoYawTest
    // }

    // dragonRotVel *= dMaxAngularRate;   // delete because Photon_Lock has MaxAngRate which is not known here

    dragonRotVel *= -1.0;

    switch(iTargetLostCounter) {
      case 0, 1:
        // do nothing, full scaling
        break;
      case 2, 3:
        dragonRotVel = dragonRotVel * 0.75;
        break;
      case 4, 5:
        dragonRotVel = dragonRotVel * 0.50;
        break;
      case 6, 7:
        dragonRotVel = dragonRotVel * 0.25;
        break;
      default:
        dragonRotVel = 0.0;
    }

    return dragonRotVel;
  }

  public double getPhotonRange() {
    return dFilteredAvgTargetRange;
  }

}
