package frc.robot.commands.SwerveDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;

public class TrackNote extends Command {
  SwerveDrive swerveDrive;

  Supplier<Double> x;
  Supplier<Double> y;

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)));

  PIDController rotController = Constants.SwerveDrive.kActiveTrackPIDController;
  PIDController headingController = Constants.SwerveDrive.kHeadingController;

  double targetHeading = 0;
  boolean stoppedRotating = false;

  int lastPipeline = 0;
  
  public TrackNote(SwerveDrive swerveDrive, Supplier<Double> x, Supplier<Double> y) {
    this.swerveDrive = swerveDrive;
    this.x = x;
    this.y = y;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    double x = this.x.get();
    double y = this.y.get();
    double rot;

    rot = rotController.calculate(LimelightHelpers.getTX(Constants.Vision.kLimelightName), 0);


    x = Math.abs(x) < Constants.SwerveDrive.kJoystickDeadband ? 0 : x;
    y = Math.abs(y) < Constants.SwerveDrive.kJoystickDeadband ? 0 : y;
    
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);

    x *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    y *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    rot *= Constants.SwerveDrive.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond);
    
    swerveDrive.driveRobotRelative(x, y, rot);
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
