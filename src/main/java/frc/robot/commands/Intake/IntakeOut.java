package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers;

public class IntakeOut extends Command {
  private Timer timer = new Timer();
  private IntakeRollers rollers;

  public IntakeOut(IntakeRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    rollers.giveToShooter();
  }

  @Override
  public void end(boolean interrupted) {
    rollers.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
