package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers;

public class IntakeAmp extends Command {
  private Timer timer = new Timer();
  private IntakeRollers rollers;

  public IntakeAmp(IntakeRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }
  
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();

  }
  
  @Override
  public void execute() {
    this.rollers.giveToShooter();
  }
  
  @Override
  public void end(boolean interrupted) {
    this.rollers.stop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
