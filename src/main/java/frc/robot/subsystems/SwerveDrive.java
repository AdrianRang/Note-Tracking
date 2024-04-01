package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Gyro.Gyro;
import lib.team3526.math.RotationalInertiaAccumulator;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveDrive extends SubsystemBase {
    // * Swerve Modules
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    // * Gyro
    Gyro gyro;

    // * Odometry
    SwerveDrivePoseEstimator odometry;

    // * Speed stats
    boolean drivingRobotRelative = false;
    ChassisSpeeds speeds = new ChassisSpeeds();

    // * Rotational Inertia Accumulator
    RotationalInertiaAccumulator rotationalInertiaAccumulator = new RotationalInertiaAccumulator(Constants.SwerveDrive.PhysicalModel.kRobotMassKg);

    // * Odometry field
    Field2d m_field = new Field2d();

    public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, Gyro gyro) {
        // Swerve Modules
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Gyro
        this.gyro = gyro;

        // Odometry
        this.odometry = new SwerveDrivePoseEstimator(
            Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
            this.getHeading(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            Constants.Field.kInitialPoseMeters,
            Constants.SwerveDrive.kEncoderStdDev,
            Constants.SwerveDrive.kVisionStdDev
        );
        
        // Reset gyro
        this.gyro.reset();

        // Configure Auto Builder
        this.configureAutoBuilder();
    }

    /**
     * Configure the auto builder for PathPlanner
     */
    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotRelativeChassisSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                Constants.SwerveDrive.Autonomous.kTranslatePIDConstants,
                Constants.SwerveDrive.Autonomous.kRotatePIDConstants,
                Constants.SwerveDrive.Autonomous.kMaxSpeedMetersPerSecond.in(MetersPerSecond),
                Constants.SwerveDrive.PhysicalModel.kWheelBase.in(Meters) / 2,
                new ReplanningConfig(true, true)
            ),
            () -> {
                if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
                return false;
            },
            this
        );
    }

    /**
     * Get the current heading of the robot
     */
    public Rotation2d getHeading() {
        return gyro.getHeading();
    }

    /**
     * Zero the heading of the robot
     */
    public void zeroHeading() {
        this.gyro.reset();
    }

    /**
     * Get the current pose of the robot
     * @return
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public void setVisionPose() {
        odometry.resetPosition(getHeading(), getModulePositions(), LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.kLimelightName));
    }

    /**
     * Reset the pose of the robot to (0, 0)
     */
    public void resetPose() {
        resetOdometry(new Pose2d());
    }

    /**
     * Reset the pose of the robot to the provided pose
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(this.getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) return this.speeds;
        else return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
    }

    /**
     * Get the target module states
     * @return
     */
    public SwerveModuleState[] getModuleTargetStates() {
        return new SwerveModuleState[]{
            frontLeft.getTargetState(),
            frontRight.getTargetState(),
            backLeft.getTargetState(),
            backRight.getTargetState()
        };
    }

    /**
     * Get the real module states
     * @return
     */
    public SwerveModuleState[] getModuleRealStates() {
        return new SwerveModuleState[]{
            frontLeft.getRealState(),
            frontRight.getRealState(),
            backLeft.getRealState(),
            backRight.getRealState()
        };
    }

    /**
     * Get the current module positions
     * @return
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),

            backLeft.getPosition(),
            backRight.getPosition(),
        };
    }

   /**
     * Set the module states
     * @param states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)></b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        SwerveModuleState[] m_moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading()));
    }

    /**
     * Drive the robot with the provided speeds <b>(FIELD RELATIVE)</b>
     * @param speeds
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    /**
     * Stop the robot (sets all motors to 0)
     */
    public void stop() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    /**
     * Angle all wheels to point inwards in an X pattern
     */
    public void xFormation() {
        this.frontLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        this.frontRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    }

    /**
     * Reset the turning encoders of all swerve modules
     */
    public void resetTurningEncoders() {
        this.frontLeft.resetTurningEncoder();
        this.frontRight.resetTurningEncoder();
        this.backLeft.resetTurningEncoder();
        this.backRight.resetTurningEncoder();
    }

    /**
     * Reset the drive encoders of all swerve modules
     */
    public void resetDriveEncoders() {
        this.frontLeft.resetDriveEncoder();
        this.frontRight.resetDriveEncoder();
        this.backLeft.resetDriveEncoder();
        this.backRight.resetDriveEncoder();
    }

    /**
     * Reset all encoders of all swerve modules
     */
    public void resetEncoders() {
        this.resetTurningEncoders();
        this.resetDriveEncoders();
    }

    /**
     * Update the odometry with the latest vision measurements
     */
    public void visionUpdate() {
        LimelightResults visionResults = LimelightHelpers.getLatestResults(Constants.Vision.kLimelightName);
        Pose2d visionBotPose = LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.kLimelightName);
        double visionLatency = (visionResults.targetingResults.latency_capture / 1000) + (visionResults.targetingResults.latency_pipeline / 1000) + (visionResults.targetingResults.latency_jsonParse / 1000);
        double captureTimestamp = Timer.getFPGATimestamp() - visionLatency;
        double mainTargetArea = LimelightHelpers.getTA(Constants.Vision.kLimelightName);

        if (visionResults.targetingResults.valid) {
            double poseDifference = odometry.getEstimatedPosition().getTranslation().getDistance(visionBotPose.getTranslation());
            if (poseDifference > Constants.Vision.kMaxPoseDifferenceMeters) return;

            double xyStdDev;
            double rotStdDev;

            if (visionResults.targetingResults.targets_Fiducials.length >= 2) {
                xyStdDev = 0.5;
                rotStdDev = 6;
            } else if (mainTargetArea > 0.8 && poseDifference < 0.5) {
                xyStdDev = 1;
                rotStdDev = 12;
            } else if (mainTargetArea > 0.1 && poseDifference < 0.3) {
                xyStdDev = 2;
                rotStdDev = 30;
            } else return;

            odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, Math.toRadians(rotStdDev)));
            this.odometry.addVisionMeasurement(visionBotPose, captureTimestamp);
        }
    }

    public void periodic() {
        // Update inertia acculumator
        rotationalInertiaAccumulator.update(this.getHeading().getRadians());

        // Update odometry
        this.odometry.update(getHeading(), getModulePositions());
        
        // Update vision measurements if cofigured
        if (Constants.SwerveDrive.kUseVisionOdometry) this.visionUpdate();

        // Update the field
        m_field.setRobotPose(this.getPose());

        // Log data
        SmartDashboard.putData("SwerveDrive/Field", this.m_field);
        SmartDashboard.putNumber("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        SmartDashboard.putNumber("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());
        SmartDashboard.putNumber("SwerveDrive/RobotRotationalInertia", rotationalInertiaAccumulator.getTotalRotationalInertia());
        SmartDashboard.putBoolean("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        // Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRobotRelativeChassisSpeeds());
        // Logger.recordOutput("SwerveDrive/ModuleRealStates", this.getModuleRealStates());
        // Logger.recordOutput("SwerveDrive/ModuleTargetStates", this.getModuleTargetStates());
    }
}
