package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.control.LazyCANSparkMax;

public class IntakeRollers extends SubsystemBase {
    private final LazyCANSparkMax rollersMotor;
    private final SparkPIDController rollersMotorPID;
    private final RelativeEncoder rollersMotorEncoder;

    private final DigitalInput limitSwitch;

    private boolean isIntaking = false;

    private double setRollerSpeed = 0.0;

    public IntakeRollers() {
        this.rollersMotor = new LazyCANSparkMax(Constants.Intake.kintakeMotorID, MotorType.kBrushless);
        this.rollersMotorPID = this.rollersMotor.getPIDController();
        PIDFConstants.applyToSparkPIDController(rollersMotorPID, Constants.Intake.kIntakePIDConstants);
        this.rollersMotorEncoder = this.rollersMotor.getEncoder();

        this.limitSwitch = new DigitalInput(Constants.Intake.kLimitSwitchPort);
    }

    public void setRollersOut() {
        this.setRollersSpeed(Constants.Intake.kIntakeOutSpeed);
    }

    public void setRollersIn() {
        this.setRollersSpeed(Constants.Intake.kIntakeInSpeed);
    }

    public void giveToShooter() {
        this.setRollersSpeed(Constants.Intake.kGiveToShooterSpeed);
    }

    public void setRollersHold() {
        this.setRollersSpeed(Constants.Intake.kIntakeHoldSpeed);
    }

    public void stop() {
        this.setRollersSpeed(0);
        this.rollersMotorPID.setReference(0, ControlType.kVelocity);
        this.setRollerSpeed = 0.0;
    }

    public void setRollersSpeed(double speed) {
        this.isIntaking = speed > 0;
        this.rollersMotor.set(speed);
    }

    public void setRollersSpeedRpm(Measure<Velocity<Angle>> rpm) {
        this.setRollerSpeed = rpm.in(RPM);
        this.isIntaking = rpm.in(RPM) > 0;
        this.rollersMotorPID.setReference(rpm.in(RPM), ControlType.kVelocity);
    }

    public void setRollersCoast() {
        SmartDashboard.putBoolean("Intake/Brake", false);
        this.rollersMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setRollersBrake() {
        SmartDashboard.putBoolean("Intake/Brake", true);
        this.rollersMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getRollersSpeed() {
        return this.rollersMotorEncoder.getVelocity();
    }

    public boolean hasPiece() {
        return !this.limitSwitch.get();
    }
}
