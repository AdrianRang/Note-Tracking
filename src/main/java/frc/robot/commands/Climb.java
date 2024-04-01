package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber climber;
  Supplier<Boolean> upSupplier;
  boolean up;

  public Climb(Climber climber, Supplier<Boolean> up) {
    this.climber = climber;
    this.upSupplier = up;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.up = upSupplier.get();
    if (this.up) climber.setClimberDown();
    else climber.setClimberUp();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
