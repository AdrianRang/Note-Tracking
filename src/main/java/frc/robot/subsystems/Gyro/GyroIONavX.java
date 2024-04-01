package frc.robot.subsystems.Gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;

public class GyroIONavX implements GyroIO {
    public AHRS gyro;

    public GyroIONavX() {
        gyro = new AHRS(I2C.Port.kMXP);
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getYaw() {
        return gyro.getAngle() % 360;
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getPitchVelocity() {
        return gyro.getRawGyroX();
    }

    public double getYawVelocity() {
        return gyro.getRawGyroZ();
    }

    public double getRollVelocity() {
        return gyro.getRawGyroY();
    }

    public double getAccelerationX() {
        return gyro.getRawAccelX();
    }

    public double getAccelerationY() {
        return gyro.getRawAccelY();
    }

    public double getAccelerationZ() {
        return gyro.getRawAccelZ();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public void reset() {
        // Reset the gyro to 0 degrees
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
                Thread.sleep(1000);
                gyro.zeroYaw();
            } catch (Exception e) {}
        }).start();
    }

    public void setYaw(double yawDeg) {
        gyro.setAngleAdjustment(yawDeg - getYaw());
    }
}
