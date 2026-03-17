/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs = kMultiTagStdDevs; // safe default; updated each cycle
    private AprilTagFieldLayout kTagLayout;
    private Pose2d lastPose = new Pose2d();

    // Simulation
    private VisionSystemSim visionSim;

    public Vision() {
        camera = new PhotonCamera(Constants.Vision.sCameraName);
        kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // PhotonVision v2026: use the 2-arg constructor (fieldTags, robotToCamera).
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(kTagLayout);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also updates {@link #curStdDevs} based on the quality of the estimate.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        SmartDashboard.putBoolean("vision/cam connected", camera.isConnected());

        // FIX 1: Call getAllUnreadResults() exactly once and reuse the list.
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        SmartDashboard.putNumber("vision/results", results.size());

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : results) {
            // FIX 2: Try coprocessor multi-tag first, fall back to single-tag.
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                //visionEst = photonEstimator.estimateSingleTagPose(result);
            }

            if (visionEst.isEmpty()) continue;

            updateEstimationStdDevs(visionEst, result.getTargets());
            lastPose = visionEst.get().estimatedPose.toPose2d();
            SmartDashboard.putBoolean("vision/has tag?", true);
        }

        if (visionEst.isEmpty()) {
            SmartDashboard.putBoolean("vision/has tag?", false);
        }

        return visionEst;
    }

    /**
     * Calculates new standard deviations based on number of tags, estimation strategy,
     * and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame.
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

            SmartDashboard.putNumber("vision/tag id", tgt.getFiducialId());
        }

        if (numTags == 0) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        avgDist /= numTags;

        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }

        // If only one tag and it's far away, trust it very little
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevs = estStdDevs;
        SmartDashboard.putNumber("vision/avg tag dist", avgDist);
        SmartDashboard.putNumber("vision/num tags", numTags);
    }

    /**
     * Returns the latest standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use with
     * {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}.
     * Should only be used when targets are visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public Pose2d lastPose() {
        return lastPose;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotSimPose);
        }
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.resetRobotPose(pose);
        }
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation() || visionSim == null) return null;
        return visionSim.getDebugField();
    }
} 
