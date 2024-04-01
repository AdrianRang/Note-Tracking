package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;

public class BasicShoot extends Command {
  Shooter m_shooter;
  IntakeRollers rollers;
  private Timer timer = new Timer();

  public BasicShoot(Shooter shooter, IntakeRollers rollers) {
    this.m_shooter = shooter;
    this.rollers = rollers;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    this.m_shooter.set(0.2);
    this.rollers.giveToShooter();
    
  }

  @Override
  public void end(boolean interrupted) {
    this.m_shooter.stop();
    this.rollers.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Shooter.kMaxShootTime;
  }
}
