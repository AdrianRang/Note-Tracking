package frc.robot.commands.SwerveDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DriveFor extends Command {
  SwerveDrive swerveDrive;

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAcceleration);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.SwerveDrive.PhysicalModel.kMaxAngularAcceleration.in(RadiansPerSecond.per(Second)));

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> rotSpeed;
  Supplier<Boolean> fieldRelative;

  Timer timer = new Timer();
  double duration;

  double lastLoop = Timer.getFPGATimestamp();
  
  public DriveFor(SwerveDrive swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldRelative, double duration) {
    this.swerveDrive = swerveDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldRelative = fieldRelative;
    this.duration = duration;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double x = xSpeed.get();
    double y = ySpeed.get();
    double rot = rotSpeed.get();

    x = Math.abs(x) < Constants.SwerveDrive.kJoystickDeadband ? 0 : x;
    y = Math.abs(y) < Constants.SwerveDrive.kJoystickDeadband ? 0 : y;
    rot = Math.abs(rot) < Constants.SwerveDrive.kJoystickDeadband ? 0 : rot;
    
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    rot = rotLimiter.calculate(rot);

    x *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    y *= Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond);
    rot *= Constants.SwerveDrive.PhysicalModel.kMaxAngularSpeed.in(RadiansPerSecond);
    
    if (this.fieldRelative.get()) {
      swerveDrive.driveFieldRelative(x, y, rot);
    } else {
      swerveDrive.driveRobotRelative(x, y, rot);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= duration;
  }
}
