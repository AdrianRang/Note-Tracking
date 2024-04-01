package frc.robot.commands.Climbers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbersDown extends Command {
  
  private final Climber leftClimber;
  private final Climber rightClimber;

  public ClimbersDown(Climber leftClimber, Climber rightClimber) {
    this.leftClimber = leftClimber;
    this.rightClimber = rightClimber;
    addRequirements(leftClimber, rightClimber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.leftClimber.setClimberDown();
    this.rightClimber.setClimberDown();
  }

  @Override
  public void end(boolean interrupted) {
    this.leftClimber.stop();
    this.rightClimber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
