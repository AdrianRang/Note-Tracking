package frc.robot.commands.Climbers;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;

public class ClimbersFullyDown extends ParallelCommandGroup {
  public ClimbersFullyDown(Climber leftClimber, Climber rightClimber) {
    addCommands(
      new ClimberFullyDown(leftClimber),
      new ClimberFullyDown(rightClimber)
    );
  }
}
