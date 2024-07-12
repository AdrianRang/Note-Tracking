package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.commands.SwerveDrive.TrackNote;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.team3526.driveControl.CustomController;

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
  
  public RobotContainer() {
    this.m_driverControllerCustom = new CustomController(0, CustomController.CustomControllerType.XBOX, CustomController.CustomJoystickCurve.LINEAR);
    this.m_operatorControllerCustom = new CustomController(1, CustomController.CustomControllerType.XBOX, CustomController.CustomJoystickCurve.LINEAR);

    this.m_frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
    this.m_frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
    this.m_backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
    this.m_backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

    this.m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));

    this.m_swerveDrive = new SwerveDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro);
 
    // Add data to SmartDashboard
    SmartDashboard.putData("ActiveTrackPID", Constants.SwerveDrive.kActiveTrackPIDController);
    SmartDashboard.putData("HeadingControllerPID", Constants.SwerveDrive.kHeadingController);
    SmartDashboard.putData("ZeroHeading", new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    SmartDashboard.putData("ResetTurningEncoders", new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()));

    // Start camera server
    CameraServer.startAutomaticCapture();

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
    controller.leftTrigger().whileTrue(new TrackNote(this.m_swerveDrive, () -> controller.getRightX(), () -> controller.getRightY()));
  }

  public Command getAutonomousCommand(){
    return null;
  }

  public Command getTeleopInitCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_swerveDrive.resetTurningEncoders())
    );
  }
}

