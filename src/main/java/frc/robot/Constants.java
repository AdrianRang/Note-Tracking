package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.team3526.constants.CTRECANDevice;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.constants.SwerveModuleOptions;
import lib.team3526.led.CANdleLEDStrip;
import lib.team3526.led.animations.PhaseAnimation;
import lib.team3526.led.animations.ShootingStarAnimation;
import lib.team3526.led.framework.HyperLEDAnimation;
import lib.team3526.utils.SwerveChassis;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
    public static final class Logging {
        public static final boolean kDebug = true;
    }

    public static final class SwerveDrive {
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

        // Active track pid constants
        public static final PIDController kActiveTrackPIDController = new PIDController(0.015, 0.0025, 0);
        public static final boolean kActiveTrackUseAprilTags = true;

        // Rotation lock PIDF Constants
        public static final PIDController kHeadingController = new PIDController(0.01, 0.0, 0.0);

        // Visiom odometry
        public static final boolean kUseVisionOdometry = true;
        public static final Matrix<N3, N1> kEncoderStdDev = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N3, N1> kVisionStdDev = VecBuilder.fill(0, 0, 0);

        // Active track pid constants
        public static final double kActiveTrackP = 0.01;
        public static final double kActiveTrackD = 0;
        public static final double kActiveTrackI = 0;

        public static final double kJoystickDeadband = 0.1;
        //! Physical model of the robot
        public static final class PhysicalModel {
            //! MAX DISPLACEMENT SPEED (and acceleration)
            public static final SendableChooser<Measure<Velocity<Distance>>> kMaxSpeedChooser = new SendableChooser<>();
            static {
                kMaxSpeedChooser.setDefaultOption("0.5 m/s", MetersPerSecond.of(0.5));
                kMaxSpeedChooser.addOption("1 m/s", MetersPerSecond.of(1));
                kMaxSpeedChooser.addOption("2 m/s", MetersPerSecond.of(2));
                kMaxSpeedChooser.addOption("3 m/s", MetersPerSecond.of(3));
                kMaxSpeedChooser.addOption("4 m/s", MetersPerSecond.of(4));
                kMaxSpeedChooser.onChange((option) -> {kMaxSpeed = option;});
                SmartDashboard.putData("SwerveDrive/MaxSpeed", kMaxSpeedChooser);
            }
            public static Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4);
            public static final double kMaxAcceleration = 3;

            //! MAX ROTATIONAL SPEED (and acceleration)
            public static final Measure<Velocity<Angle>> kMaxAngularSpeed = RadiansPerSecond.of(1 * (2 * Math.PI));
            public static final Measure<Velocity<Velocity<Angle>>> kMaxAngularAcceleration = RadiansPerSecond.per(Second).of(kMaxAngularSpeed.in(RadiansPerSecond));

            // Drive wheel diameter
            public static final Measure<Distance> kWheelDiameter = Inches.of(4);

            // Gear ratios
            public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
            public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

            // Conversion factors (Drive Motor)
            public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * (kWheelDiameter.in(Meters) / 2) * 2 * Math.PI;
            public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

            // Conversion factors (Turning Motor)
            public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI;
            public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0;

            // Robot Without bumpers measures
            public static final Measure<Distance> kTrackWidth = Inches.of(23.08);
            public static final Measure<Distance> kWheelBase = Inches.of(22.64);
    
            // Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));

            // Rotational inertia constants
            public static final double kRobotMassKg = 46;

            // Standard deviations
            public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0, 0, 0);
        }

        //! Swerve modules configuration
        //! Swerve modules configuration
        public static final class SwerveModules {
            //! PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(0.5);

            //! Global offset
            public static final Measure<Angle> kGlobalOffset = Degrees.of(0);

            //! Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(11, "*"))
                .setDriveMotorID(22)
                .setTurningMotorID(21)
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(12, "*"))
                .setDriveMotorID(24)
                .setTurningMotorID(23)
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
                .setDriveMotorID(26)
                .setTurningMotorID(25)
                .setName("Back Left");

            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(14, "*"))
                .setDriveMotorID(28)
                .setTurningMotorID(27)
                .setName("Back Right");
        }

        //! AUTONOMOUS
        public static final class Autonomous {
            public static final PIDConstants kTranslatePIDConstants = new PIDConstants(1.45, 0.5, 0.0);
            public static final PIDConstants kRotatePIDConstants = new PIDConstants(10, 0.0, 0.0);
            public static final Measure<Velocity<Distance>> kMaxSpeedMetersPerSecond = MetersPerSecond.of(1);
        }
    }

    //! FIELD
    public static final class Field {
        public static final Pose2d kInitialPoseMeters = new Pose2d(new Translation2d(1, 2), new Rotation2d(0, 0));
        public static final Pose2d kBlueSpeakerPoseMeters = new Pose2d(new Translation2d(0, 5.55), Rotation2d.fromDegrees(0));
        public static final Pose2d kRedSpeakerPoseMeters = new Pose2d(new Translation2d(17, 5.55), Rotation2d.fromDegrees(0));
    }

    //! VISION
    public static final class Vision {
        public static final double kMaxPoseDifferenceMeters = 2;
        public static final String kLimelightName = "limelight";
        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public static final Transform3d kCameraPose = new Transform3d(new Translation3d(0.5, 0.5, 0.3), new Rotation3d(0, -2, 0));
        public static final int kOdometryPipeline = 0;
        public static final int kActiveTrackPipeline = 1;
    }

    //* INTAKE
    public static final class Intake {

    //////////////////////////////////////////////// ROLLERS ////////////////////////////////////////////////

        // Intake motor config
        public static final int kintakeMotorID = 36;
        public static final PIDFConstants kIntakePIDConstants = new PIDFConstants(0.00001, 0.0, 0.0,0.0001);
        public static final double kHasPieceCurrentThreshold = 20;
        public static final double kHasPieceTimeThreshold = 0.2;

        // Intake gear ratio
        public static final double kIntakeRollersGearRatio = 5.0/1.0;

        // Speeds
        public static final double kIntakeOutSpeed = -0.5;
        public static final Measure<Velocity<Angle>> kIntakeOutSpeedRpm = RPM.of(kIntakeOutSpeed*kIntakeRollersGearRatio);
        public static final double kIntakeInSpeed = 0.5;
        public static final Measure<Velocity<Angle>> kIntakeInSpeedRpm = RPM.of(kIntakeInSpeed*kIntakeRollersGearRatio);
        public static final double kIntakeHoldSpeed = 0.05;
        public static final Measure<Velocity<Angle>> kIntakeHoldSpeedRpm = RPM.of(kIntakeHoldSpeed*kIntakeRollersGearRatio);
        public static final double kGiveToShooterSpeed = -0.6;
        public static final Measure<Velocity<Angle>> kGiveToShooterSpeedRpm = RPM.of(kGiveToShooterSpeed*kIntakeRollersGearRatio);
        public static final double kMaxLifterSpeed = 0.5;
        public static final double kLifterDownSpeed = -0.25;
        public static final double kLifterUpSpeed = 0.25;

        // Lifter encoder 
        public static final int kLifterEncoderPort = 0;
        public static final double kLifterEncoderOffset = 0.367;

        // Intake times
        public static final double kMaxOuttakeTime = 3;
        public static final double kMaxIntakeTime = 6;

        public static final int kLimitSwitchPort = 1;
        
    ///////////////////////////////////////////////// LIFTER /////////////////////////////////////////////////
        
        // Lifter motor config
        public static final int kLifterMotorID = 37;
        public static final ArmFeedforward kLifterFeedforward = new ArmFeedforward(0.0, 0, 0.0);
        public static final Constraints kLifterConstraints = new Constraints(42, 38);
        public static final ProfiledPIDController kLifterPIDController = new ProfiledPIDController(3.15, 0.0, 0.0, kLifterConstraints);

        public static final class Physical {
            public static final Measure<Angle> kLifterMaxHeight = Radians.of((37/36)*Math.PI);
            public static final Measure<Angle> kLifterMinHeight = Radians.of(0);

            public static final Measure<Angle> kShooterAngle = Radians.of(0);
            public static final Measure<Angle> kAmplifierPrevAngle = Degrees.of(40);
            public static final Measure<Angle> kAmplifierFinalAngle = Degrees.of(85);
            public static final Measure<Angle> kGroundAngle = Degrees.of(176);
        }
    }

    //! SHOOTER
    public static final class Shooter {
        // Shooter motor config
        public static final int kLeftShooterMotorID = 30;
        public static final int kRightShooterMotorID = 31;

        // Shooter motor rpm conversion
        public static final double kShooterGearRatio = 1.0 / 1.0;

        // Shooter speeds
        public static final double kShooterMaxSpeed = 1;
        public static final double kShooterSpeakerLeftSpeed = kShooterMaxSpeed;
        public static final double kShooterSpeakerRightSpeed = kShooterMaxSpeed * 0.95;

        // Shooter motor time
        public static final double kMaxShootTime = 4;

        // Shooter angle
        public static final Rotation2d kRobotAngle = Rotation2d.fromDegrees(180);
    }

    //! CLIMBER
    public static final class Climber {
        // Climber motor config
        public static final int kLeftClimberMotorID = 33;
        public static final int kRightClimberMotorID = 32;

        // Climber speed
        public static final double kClimberUpSpeed = 0.8;
        public static final double kClimberDownSpeed = -0.75;
        public static final double kClimberHoldSpeed = kClimberDownSpeed / 2;

        // Climber motor encoder conversion
        public static final double kClimber_RotationToCentimeters = 1 / 16 / 3 * 31;

        // Max current (Used for reseting the climber)
        public static final double kMaxCurrent = 20;

        // Climber encoder
        public static final double kLedExtensionThreshold = 1;
        public static final double kMaxExtension = 31;
    }

    public static final class CANdle {
        public static final int kCANdleID = 0;

        public static final CTRECANDevice kCANdle = new CTRECANDevice(41, "*");


        public static final LEDStripType kLEDStripType = LEDStripType.GRB;
        public static final double kLEDBrightness = 0.25;

        public static final CANdleLEDStrip kLeftClimber = new CANdleLEDStrip(0, 9);
        public static final CANdleLEDStrip kRightClimber = new CANdleLEDStrip(9, 18);
    }

    public static final class HyperLEDs {
        public static final HyperLEDAnimation kDefaultAnimation = new PhaseAnimation(0, 0, 255, 1);
        public static final HyperLEDAnimation kShootAnimation = new ShootingStarAnimation(120, 100, 100, 1, 0.5);
    }
}
