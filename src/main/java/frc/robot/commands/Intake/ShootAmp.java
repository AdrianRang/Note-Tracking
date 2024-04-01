package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LedsSubsystem;

public class ShootAmp extends Command {

  private final IntakeRollers rollers;
  private final IntakeLifter lifter;
  private final LedsSubsystem leds;
  private final Timer timer = new Timer();

  public ShootAmp(IntakeRollers rollers, IntakeLifter lifter, LedsSubsystem leds) {
    this.rollers = rollers;
    this.lifter = lifter;
    this.leds = leds;
    addRequirements(rollers, lifter, leds);
  }

  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
    this.leds.setLeds("#ad03fc");
  }

  @Override
  public void execute() {
    this.lifter.setLifterAngle(Constants.Intake.Physical.kAmplifierFinalAngle);
    if (this.timer.get()>0.3) {    
      this.rollers.setRollersSpeed(-0.43);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.rollers.stop();
    this.timer.stop();
    this.lifter.setLifterAngle(Constants.Intake.Physical.kShooterAngle);
    this.leds.turnOff();
  }

  @Override
  public boolean isFinished() {
    return (this.timer.get()>1.0);
  }
}
