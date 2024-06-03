package frc.robot.commands.Climbers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import lib.team3526.commands.RunForCommand;

public class ClimbersReset extends SequentialCommandGroup {
  public ClimbersReset(Climber leftClimber, Climber rightClimber) {
    addCommands(
      new ClimbersFullyDown(leftClimber, rightClimber),
      new RunForCommand(new ClimbersUp(leftClimber, rightClimber), 0.25),
      new RunForCommand(new ClimbersDown(leftClimber, rightClimber), 0.5),
      new InstantCommand(() -> leftClimber.resetEncoder()),
      new InstantCommand(() -> rightClimber.resetEncoder())
    );
  }
}
