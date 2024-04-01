package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter;

public class LifterAmp extends Command {
  IntakeLifter intake;

  public LifterAmp(IntakeLifter intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.setLifterAngle(Constants.Intake.Physical.kAmplifierPrevAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
