// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Robot extends LoggedRobot {
  final boolean REPLAY = false;

  PS4Controller controller = new PS4Controller(0);

  Pose2d pose = new Pose2d();

  VisionSystemSim visionSystem = new VisionSystemSim("atlas vision");
  PhotonCamera camera = new PhotonCamera("camera");
  PhotonCameraSim cameraSim;
  Transform3d ROBOT_TO_CAMERA = new Transform3d(0, 0, Units.inchesToMeters(20), new Rotation3d(0, -Units.degreesToRadians(10), Math.PI));
  LoggedDashboardNumber pitch = new LoggedDashboardNumber("pitch", 10);
  LoggedDashboardNumber height = new LoggedDashboardNumber("height", 20);
  double pitchValue = pitch.get();
  double heightValue = height.get();

  VisionInputsAutoLogged visionInputs = new VisionInputsAutoLogged();

  boolean shouldVibrate = true;
  boolean robotRelative = false;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "Atlas Vision Sim"); // Set a metadata value

    if (!REPLAY) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(true); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    LogTable.disableProtobufWarning();

    visionSystem.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(320, 240, Rotation2d.fromDegrees(80.0));
    properties.setAvgLatencyMs(100.0);
    properties.setCalibError(0.25, 0.08);
    properties.setFPS(10);
    cameraSim = new PhotonCameraSim(camera, properties);
    cameraSim.enableDrawWireframe(true);
    visionSystem.addCamera(cameraSim, ROBOT_TO_CAMERA);
  }

  @Override
  public void robotPeriodic() {
    visionInputs.result = camera.getLatestResult();
    Logger.processInputs("vision", visionInputs);

    if (shouldVibrate && visionInputs.result.hasTargets()) {
      controller.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      controller.setRumble(RumbleType.kBothRumble, 0);
    }

    if (controller.getR1ButtonPressed()) {
      System.out.println();
      for (var target : visionInputs.result.targets) {
        System.out.print(target.getYaw() + " ");
      }
    }

    if (controller.getCircleButtonPressed()) {
      shouldVibrate = !shouldVibrate;
    }

    if (controller.getTriangleButtonPressed()) {
      robotRelative = !robotRelative;
    }

  }

  @Override
  public void simulationPeriodic() {
    double forw = -0.03 * MathUtil.applyDeadband(controller.getLeftY(), 0.1);
    double strafe = -0.03 * MathUtil.applyDeadband(controller.getLeftX(), 0.1);
    double rot = -0.02 * MathUtil.applyDeadband(controller.getRightX(), 0.1);
    ChassisSpeeds speeds = new ChassisSpeeds(forw, strafe, rot);
    if (!robotRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
    } else {
      speeds.vxMetersPerSecond = -speeds.vxMetersPerSecond;
      speeds.vyMetersPerSecond = -speeds.vyMetersPerSecond;
    }
    pose = pose.exp(new Twist2d(
        speeds.vxMetersPerSecond, 
        speeds.vyMetersPerSecond, 
        speeds.omegaRadiansPerSecond));
    Logger.recordOutput("pose", pose);

    if (pitchValue != pitch.get() || heightValue != height.get()) {
      pitchValue = pitch.get();
      heightValue = height.get();
      ROBOT_TO_CAMERA = new Transform3d(0, 0, Units.inchesToMeters(heightValue), new Rotation3d(0, -Units.degreesToRadians(pitchValue), Math.PI));
      visionSystem.adjustCamera(cameraSim, ROBOT_TO_CAMERA);
    }

    visionSystem.update(pose);
  }

  @AutoLog
  public static class VisionInputs {
    PhotonPipelineResult result;
  }

}
