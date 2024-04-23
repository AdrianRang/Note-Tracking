package frc.robot.commands.SwerveDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;

public class DriveSwerve extends Command {
  SwerveDrive swerveDrive;

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)));

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> rotSpeed;
  Supplier<Boolean> fieldRelative;
  Supplier<Boolean> trackingSpeaker;

  PIDController rotController = Constants.SwerveDrive.kActiveTrackPIDController;
  PIDController headingController = Constants.SwerveDrive.kHeadingController;

  double targetHeading = 0;
  boolean stoppedRotating = false;

  int lastPipeline = 0;
  
  public DriveSwerve(SwerveDrive swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldRelative, Supplier<Boolean> trackingSpeaker) {
    this.swerveDrive = swerveDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldRelative = fieldRelative;
    this.trackingSpeaker = trackingSpeaker;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    if (trackingSpeaker.get()) {
      this.lastPipeline = (int)LimelightHelpers.getCurrentPipelineIndex(Constants.Vision.kLimelightName);
      if (Constants.SwerveDrive.kActiveTrackUseAprilTags) LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightName, Constants.Vision.kActiveTrackPipeline);
      else LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightName, Constants.Vision.kOdometryPipeline);
    }
  }

  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot;

    if (trackingSpeaker.get()) {
      if (Constants.SwerveDrive.kActiveTrackUseAprilTags) {
        rot = rotController.calculate(LimelightHelpers.getTX(Constants.Vision.kLimelightName), 0);
      } else {
        boolean isBlueAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(
          (isBlueAlliance ? Constants.Field.kBlueSpeakerPoseMeters.getY() : Constants.Field.kRedSpeakerPoseMeters.getY()) - swerveDrive.getPose().getTranslation().getY(),
          (isBlueAlliance ? Constants.Field.kBlueSpeakerPoseMeters.getX() : Constants.Field.kRedSpeakerPoseMeters.getX()) - swerveDrive.getPose().getTranslation().getX()
        ) + Math.PI).rotateBy(Constants.Shooter.kRobotAngle);
        rot = rotController.calculate(swerveDrive.getHeading().getDegrees(), targetAngle.getDegrees());
      }
    } else {
      rot = rotSpeed.get();
    }

    x = Math.abs(x) < Constants.SwerveDrive.kJoystickDeadband ? 0 : x;
    y = Math.abs(y) < Constants.SwerveDrive.kJoystickDeadband ? 0 : y;
    rot = Math.abs(rot) < Constants.SwerveDrive.kJoystickDeadband ? 0 : rot;
    
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);

    x *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    y *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    rot *= Constants.SwerveDrive.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond);
    
    if (this.fieldRelative.get()) swerveDrive.driveFieldRelative(x, y, rot);
    else swerveDrive.driveRobotRelative(x, y, rot);
  }

  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex(Constants.Vision.kLimelightName, lastPipeline);
    swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
