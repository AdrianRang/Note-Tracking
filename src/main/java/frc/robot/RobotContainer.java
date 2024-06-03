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
import frc.robot.commands.Climbers.ClimbersFullyDown;
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
import lib.team3526.led.animations.PhaseAnimation;
import lib.team3526.led.framework.HyperAddressableLEDSegment;
import lib.team3526.led.framework.HyperAddressableLEDStrip;

public class RobotContainer {
  // * Controller
  private final CustomController m_driverControllerCustom;
  private final CustomController m_operatorControllerCustom;

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
  public static final HyperAddressableLEDSegment shooterLeds = new HyperAddressableLEDSegment(10);
  public static final HyperAddressableLEDStrip m_ledStrip = new HyperAddressableLEDStrip(0, shooterLeds);


  // * Autonomous Chooser
  SendableChooser<Command> autonomousChooser;
  
  public RobotContainer() {
    this.m_driverControllerCustom = new CustomController(0, CustomController.CustomControllerType.XBOX, CustomController.CustomJoystickCurve.LINEAR);
    this.m_operatorControllerCustom = new CustomController(1, CustomController.CustomControllerType.XBOX, CustomController.CustomJoystickCurve.LINEAR);

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

    RobotContainer.shooterLeds.setDefaultAnimation(new PhaseAnimation(0, 0, 255, 1)::provider);
    RobotContainer.m_ledStrip.leds.setBitTiming(250, 600, 600, 300);
    RobotContainer.m_ledStrip.leds.setSyncTime(300);

    // Register the named commands for autonomous
    NamedCommands.registerCommands(new HashMap<String, Command>() {{
      put("IntakeIn", new RunForCommand(new IntakeIn(m_rollers), 1));
      put("IntakeOut", new RunForCommand(new IntakeOut(m_rollers), 0.25));

      put("Shoot", new RunForCommand(new Shoot(m_shooter, m_rollers, m_leds), 1));

      put("LifterFloor", new RunForCommand(new LifterFloor(m_intake), 1));
      put("LifterShooter", new RunForCommand(new LifterShooter(m_intake), 1));

      put("PickUpPiece", new RunForCommand(new PickUpPiece(m_rollers, m_intake, m_leds), 5));

      put("ClimbersFullyDown", new RunForCommand(new ClimbersFullyDown(m_leftClimber, m_rightClimber), 5));

      put("AutoAim", new RunForCommand(new DriveSwerve(m_swerveDrive, () -> 0.0, () -> 0.0, () -> 0.0, () -> true, () -> true), 2));
    }});
 
    // Add data to SmartDashboard
    SmartDashboard.putData("ActiveTrackPID", Constants.SwerveDrive.kActiveTrackPIDController);
    SmartDashboard.putData("HeadingControllerPID", Constants.SwerveDrive.kHeadingController);
    SmartDashboard.putData("ZeroHeading", new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    SmartDashboard.putData("ResetPose", new InstantCommand(() -> m_swerveDrive.resetPose()));
    SmartDashboard.putData("SetVisionPose", new InstantCommand(() -> m_swerveDrive.setVisionPose()));
    SmartDashboard.putData("ResetTurningEncoders", new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()));
    SmartDashboard.putData("Reset Climbers", new RunForCommand(new ClimbersFullyDown(m_leftClimber, m_rightClimber), 5));

    // Start camera server
    CameraServer.startAutomaticCapture();

    // Autonomous chooser
    this.autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", this.autonomousChooser);

    configureBindings(m_driverControllerCustom, true, false);
    configureBindings(m_operatorControllerCustom, false, true);

    m_swerveDrive.resetTurningEncoders();
  }

  private void configureBindings(CustomController controller, boolean bindDrivetrain, boolean bindSubsystems) {
    if (bindDrivetrain) {
      this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
          m_swerveDrive,
          () -> -controller.getLeftY(),
          () -> -controller.getLeftX(),
          () -> -controller.getRightX(),
          () -> true,
          () -> false
        )
      );
    }
    
    controller.rightStickButton().onTrue(new InstantCommand(() -> this.m_swerveDrive.zeroHeading()));

    // Set pipeline to apriltags high framerate
    if (bindSubsystems) {
      controller.rightTrigger().whileTrue(new SpinShooter(this.m_shooter, this.m_leds));
      controller.rightTrigger().onFalse(new Shoot(this.m_shooter, this.m_rollers, this.m_leds));
  
      controller.leftTrigger().whileTrue(new IntakeOut(this.m_rollers));
  
      controller.povLeft().whileTrue(new LifterAmp(m_intake));
      controller.povLeft().onFalse(new ShootAmp(this.m_rollers, this.m_intake, this.m_leds));
  
      controller.rightBumper().whileTrue(new LifterFloor(this.m_intake));
      controller.leftBumper().whileTrue(new LifterShooter(this.m_intake));
  
      controller.povUp().whileTrue(new ClimbersUp(this.m_leftClimber, this.m_rightClimber));
      controller.povDown().whileTrue(new ClimbersDown(this.m_leftClimber, this.m_rightClimber));
  
      controller.bottomButton().toggleOnTrue(new PickUpPiece(this.m_rollers, this.m_intake, this.m_leds));
    }
  }

  public Command getAutonomousCommand() {
    return this.autonomousChooser.getSelected();
  }

  public Command getTeleopInitCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()),
      new InstantCommand(() -> m_swerveDrive.setVisionPose())
    );
  }
}
