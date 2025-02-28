package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralatorSubsystem extends SubsystemBase {
    public static class Settings {

    }

    private TalonFX mTalonArm, mTalonWrist, mTalonRoller;
    private CANcoder mCANcoderArm, mCANcoderWrist;

    @Override
    public void periodic() {

    }

    public void setWristTarget() {

    }

    public void setArmTarget() {

    }

    // Returns the angle of the wrist relative to the ground
    public Rotation2d getWristPos() {
        return null;
    }

    // Returns the angle of the arm relative to the ground
    public Rotation2d getArmPos() {
        return null;
    }

    // Returns the angle of the wrist relative to the arm. The wrist has a soft stop which moves based on its position
    // relative to the arm - this function should be used to ensure we don't violate that constraint.
    public Rotation2d getWristPosRelativeArm() {
        return null;
    }
}
