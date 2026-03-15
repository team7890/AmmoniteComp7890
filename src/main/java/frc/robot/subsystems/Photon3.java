// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;

// import static frc.robot.Constants.Vision.*;


// public class Photon3 extends SubsystemBase {
//   private final PhotonCamera objDragonCamera;
//  /// Can change in with Train///
 
// private final PhotonPoseEstimator objPhotonPoseEstimator;
// private Matrix <N3,N1> matCurStdDevs;
// private final EstimatedConsumer objEstimateConsumer;

//   /** Creates a new Photon. */
// public Photon3(EstimatedConsumer objEstimatedConsumer_in) {
//   objEstimateConsumer = objEstimatedConsumer_in;
//   objDragonCamera = new PhotonCamera(sCameraName);
//   objPhotonPoseEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
// }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     Optional<EstimatedRobotPose> visionEst = Optional.empty();
//     for (var result : objDragonCamera.getAllUnreadResults()) {
//       visionEst = objPhotonPoseEstimator.estimateCoprocMultiTagPose(result);
//       if(visionEst.isEmpty()) {
//         visionEst = objPhotonPoseEstimator.estimateLowestAmbiguityPose(result);
//       }
//       updateEstimationStdDevs(visionEst, result.getTargets());

//       visionEst.ifPresent(
//         est -> {
//           var estStdDevs = getEstimationStdDevs();
//           objEstimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
//         }
//       );
//     }
//   }

// public Matrix<N3, N1>getEstimationStdDevs(){
//   return matCurStdDevs;
// }

// private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
//   if (estimatedPose.isEmpty()){
//     matCurStdDevs =kSingleTagStdDevs;
//   }
//   else {
//     var estStdDevs = kSingleTagStdDevs;
//     int numTags = 0;
//     double avgDist = 0;

//     for (var tgt : targets){
//       var tagPose = objPhotonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//       if (tagPose.isEmpty()) continue;
//       numTags++;
//       avgDist +=
//         tagPose
//           .get()
//           .toPose2d()
//           .getTranslation()
//           .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      
//     }
  
//     if(numTags == 0) {
//       matCurStdDevs = kSingleTagStdDevs;
//     }
//     else {
//       avgDist /= numTags;
      
//       if (numTags > 1) estStdDevs = kMultiTagStdDevs;
//       if (numTags == 1 && avgDist > 4)
//           estStdDevs = VecBuilder.fill(Double.MAX_VALUE, 
//             Double.MAX_VALUE, Double.MAX_VALUE);
//         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
//         matCurStdDevs = estStdDevs;
//     }
//   }
// }

// @FunctionalInterface
// public static interface EstimatedConsumer {
//   public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
// }



// }


