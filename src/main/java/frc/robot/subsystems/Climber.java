package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.team3526.control.LazyCANSparkMax;

public class Climber extends SubsystemBase {
    LazyCANSparkMax climberMotor;
    RelativeEncoder climberEncoder;
    String name;

    public Climber(int motorID, String name) {
        this.name = name;
        this.climberMotor = new LazyCANSparkMax(motorID, MotorType.kBrushless);
        this.climberEncoder = this.climberMotor.getEncoder();
        //this.climberEncoder.setPositionConversionFactor(Constants.Climber.kClimber_RotationToCentimeters);
        //this.climberEncoder.setInverted(true);
    }

    public void set(double speed) {
        climberMotor.set(speed);
    }

    public void setClimberUp() {
        climberMotor.set(Constants.Climber.kClimberUpSpeed);
    }

    public void setClimberDown() {
        climberMotor.set(Constants.Climber.kClimberDownSpeed);
    }

    public void setClimberHold() {
        climberMotor.set(Constants.Climber.kClimberHoldSpeed);
    }

    public void stop() {
        climberMotor.set(0);
    }

    public Measure<Distance> getExtension() {
        return Centimeters.of(this.climberEncoder.getPosition());
    }

    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }

    public double getCurrent() {
        return climberMotor.getOutputCurrent();
    }
}
