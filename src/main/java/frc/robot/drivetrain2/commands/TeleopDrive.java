package frc.robot.drivetrain2.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.crevolib.io.JoystickConfig;
import frc.robot.drivetrain2.Drivetrain;
import frc.robot.drivetrain2.DrivetrainConfig;
import frc.robot.drivetrain2.DrivetrainConfig.DriveConstants;
import frc.robot.drivetrain2.DrivetrainConfig.DriveConstants.*;;

public class TeleopDrive extends Command {
    private final Drivetrain drivetrain;
    private double velocityX, velocityY, velocityRotational;

    /**
     *
     * @param translationSupplier translation demand, magnitude should be of interval [-1, 1], percent of max translational velocity
     * @param rotationSupplier interval from [-1, 1], percent of max angular velocity
     * @param isFieldRelative field relative or robot centric
     * @param rotationOffset offset for the robot's center of rotation
     */
    public TeleopDrive(double velocityX, double velocityY, double velocityRotational) {
        drivetrain = Drivetrain.getInstance();
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.velocityRotational = velocityRotational;

        addRequirements(drivetrain);
    }


    @Override
    public void execute() {
        Drivetrain.getInstance().applyRequest(() -> 
            DriveConstants.drive.withVelocityX(velocityX)
            .withVelocityY(velocityY).withRotationalRate(velocityRotational)
        );
    }
}
