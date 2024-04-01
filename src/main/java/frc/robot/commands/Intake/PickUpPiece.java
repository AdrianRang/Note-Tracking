package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LedsSubsystem;
import lib.team3526.utils.LimelightLED;

public class PickUpPiece extends Command {
  private IntakeRollers rollers;
  private IntakeLifter lifter;
  private LedsSubsystem leds;

  public PickUpPiece(IntakeRollers rollers, IntakeLifter lifter, LedsSubsystem leds) {
    this.rollers = rollers;
    this.lifter = lifter;
    this.leds = leds;
    addRequirements(rollers, lifter, leds);
  }

  @Override
  public void initialize() {
    this.leds.blinkLeds("#fc8b00");
    this.lifter.setLifterAngle(Constants.Intake.Physical.kGroundAngle);
  }
  
  @Override
  public void execute() {
    this.rollers.setRollersIn();
  }

  @Override
  public void end(boolean interrupted) {
    this.lifter.setLifterAngle(Constants.Intake.Physical.kShooterAngle);
    this.rollers.stop();
    this.leds.blinkLeds("#00ff00");
    LimelightLED.blinkLeds(Constants.Vision.kLimelightName);
  }

  @Override
  public boolean isFinished() {
    return this.rollers.hasPiece();
  }
}
