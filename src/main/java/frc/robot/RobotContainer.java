// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Climbers.ClimbersDown;
import frc.robot.commands.Climbers.ClimbersUp;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.LifterAmp;
import frc.robot.commands.Intake.LifterFloor;
import frc.robot.commands.Intake.LifterShooter;
import frc.robot.commands.Intake.PickUpPiece;
import frc.robot.commands.Intake.ShootAmp;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.SpinShooter;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeLifter;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.team3526.commands.RunForCommand;
import lib.team3526.driveControl.CustomController;

public class RobotContainer {
  // * Controller
  private final CustomController m_driverControllerCustom;

  // *  Swerve Modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  
  // * Gyroscope
  private final Gyro m_gyro;
  
  // * Swerve Drive
  private final SwerveDrive m_swerveDrive;

  // * Intake
  private final IntakeLifter m_intake;
  private final IntakeRollers m_rollers;

  // * Shooter
  private final Shooter m_shooter;

  // * Climbers
  private final Climber m_leftClimber;
  private final Climber m_rightClimber;

  // * LEDs
  private final LedsSubsystem m_leds;

  // * Autonomous Chooser
  SendableChooser<Command> autonomousChooser;
  
  public RobotContainer() {
    this.m_driverControllerCustom = new CustomController(0, CustomController.CustomControllerType.XBOX, CustomController.CustomJoystickCurve.LINEAR);

    this.m_frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
    this.m_frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
    this.m_backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
    this.m_backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

    this.m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));

    this.m_swerveDrive = new SwerveDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro);

    this.m_intake =  new IntakeLifter();
    this.m_rollers = new IntakeRollers();

    this.m_shooter = new Shooter();

    this.m_leftClimber = new Climber(Constants.Climber.kLeftClimberMotorID, "LeftClimber");
    this.m_rightClimber = new Climber(Constants.Climber.kRightClimberMotorID, "RightClimber");

    this.m_leds = new LedsSubsystem(Constants.CANdle.kCANdle);
    this.m_leds.turnOff();

    // Register the named commands for autonomous
    NamedCommands.registerCommands(new HashMap<String, Command>() {{
      put("IntakeIn", new RunForCommand(new IntakeIn(m_rollers), 1));
      put("IntakeOut", new RunForCommand(new IntakeOut(m_rollers), 0.25));

      put("Shoot", new RunForCommand(new Shoot(m_shooter, m_rollers, m_leds), 1));

      put("LifterFloor", new RunForCommand(new LifterFloor(m_intake), 1));
      put("LifterShooter", new RunForCommand(new LifterShooter(m_intake), 1));

      put("PickUpPiece", new RunForCommand(new PickUpPiece(m_rollers, m_intake, m_leds), 5));
    }});
 
    // Add data to SmartDashboard
    SmartDashboard.putData("ActiveTrackPID", Constants.SwerveDrive.kActiveTrackPIDController);
    SmartDashboard.putData("ZeroHeading", new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    SmartDashboard.putData("ResetPose", new InstantCommand(() -> m_swerveDrive.resetPose()));
    SmartDashboard.putData("SetVisionPose", new InstantCommand(() -> m_swerveDrive.setVisionPose()));
    SmartDashboard.putData("ResetTurningEncoders", new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()));

    // Start camera server
    CameraServer.startAutomaticCapture();

    // Autonomous chooser
    this.autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", this.autonomousChooser);

    configureBindings();

    m_swerveDrive.resetTurningEncoders();
  }

  private void configureBindings() {
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -this.m_driverControllerCustom.getLeftY(),
        () -> -this.m_driverControllerCustom.getLeftX(),
        () -> -this.m_driverControllerCustom.getRightX(),
        () -> true,
        () -> this.m_driverControllerCustom.topButton().getAsBoolean()
      )
    );

    this.m_driverControllerCustom.bottomButton().toggleOnTrue(new PickUpPiece(this.m_rollers, this.m_intake, this.m_leds));

    this.m_driverControllerCustom.rightTrigger().whileTrue(new SpinShooter(this.m_shooter, this.m_leds));
    this.m_driverControllerCustom.rightTrigger().onFalse(new Shoot(this.m_shooter, this.m_rollers, this.m_leds));

    this.m_driverControllerCustom.leftTrigger().whileTrue(new IntakeOut(this.m_rollers));

    this.m_driverControllerCustom.povLeft().whileTrue(new LifterAmp(m_intake));
    this.m_driverControllerCustom.povLeft().onFalse(new ShootAmp(this.m_rollers, this.m_intake, this.m_leds));

    this.m_driverControllerCustom.rightBumper().whileTrue(new LifterFloor(this.m_intake));
    this.m_driverControllerCustom.leftBumper().whileTrue(new LifterShooter(this.m_intake));

    this.m_driverControllerCustom.povUp().whileTrue(new ClimbersUp(this.m_leftClimber, this.m_rightClimber));
    this.m_driverControllerCustom.povDown().whileTrue(new ClimbersDown(this.m_leftClimber, this.m_rightClimber));

    this.m_driverControllerCustom.rightStickButton().onTrue(new InstantCommand(() -> this.m_swerveDrive.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return this.autonomousChooser.getSelected();
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()),
    //   this.autonomousChooser.getSelected()
    // );
  }

  public Command getTeleopInitCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()),
      new InstantCommand(() -> m_swerveDrive.setVisionPose())
    );
  }
}
