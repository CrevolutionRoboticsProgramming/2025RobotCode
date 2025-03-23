package frc.robot.vision.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.vision.VisionConfig.*;
import static frc.robot.vision.VisionConfig.AlignmentConfig.*;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.PoseEstimatorSubsystem;
import frc.robot.vision.VisionConfig;



public class AutoAlign extends Command {
    private double x;
    private double y;
    private LinearVelocity kLineupSpeed = MetersPerSecond.of(.1);
    private PhotonCamera leftCam = new PhotonCamera(CAM_NAMES[0]); 
    private int tagID;

    private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_VELOCITY.in(MetersPerSecond),
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_TRANSLATION_ACCELERATION.in(MetersPerSecondPerSecond));
            
    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                        VisionConfig.AlignmentConfig.MAX_ALIGN_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));


    private static final ProfiledPIDController xDistanceController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.X_kP,
                                        VisionConfig.AlignmentConfig.X_kI,
                                        VisionConfig.AlignmentConfig.X_kD,
                                        TRANSLATION_CONSTRAINTS);

    private static final ProfiledPIDController yDistanceController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.Y_kP,
                                        VisionConfig.AlignmentConfig.Y_kI,
                                        VisionConfig.AlignmentConfig.Y_kD,
                                        TRANSLATION_CONSTRAINTS);

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
                                        VisionConfig.AlignmentConfig.THETA_kP,
                                        VisionConfig.AlignmentConfig.THETA_kI,
                                        VisionConfig.AlignmentConfig.THETA_kD,
                                        THETA_CONSTRAINTS);    
        
    private static Pose2d currentPose;
    private static Pose2d targetPose;

    public AutoAlign(boolean left) {
        //set tolerances of all PID controllers
        xDistanceController.setTolerance(VisionConfig.AlignmentConfig.DISTANCE_TOLERANCE.in(Meters));
        xDistanceController.setIntegratorRange(-.15, .15);
        yDistanceController.setTolerance(VisionConfig.AlignmentConfig.LATERAL_TOLERANCE.in(Meters));
        yDistanceController.setIntegratorRange(-.15, .15);
        thetaController.enableContinuousInput(Units.degreesToRadians(-180), Units.degreesToRadians(180));

        currentPose = PoseEstimatorSubsystem.getInstance().getCurrentPose();
        targetPose = getTargetReefPose(left);

        xDistanceController.reset(0);
        yDistanceController.reset(0);
        thetaController.reset(0);

        addRequirements(CommandSwerveDrivetrain.getInstance());
    }

    @Override
    public void execute() {
        currentPose = PoseEstimatorSubsystem.getInstance().getCurrentPose();
        // ChassisSpeeds currentSpeeds = CommandSwerveDrivetrain.getInstance().getState().Speeds;
        x = currentPose.getTranslation().getX();
        y = currentPose.getTranslation().getY();
        // double Xvel = currentSpeeds.vxMetersPerSecond;
        // double Yvel = currentSpeeds.vyMetersPerSecond;
        double XOutput = 0.15 * kLineupSpeed.in(MetersPerSecond) * 0.95 * xDistanceController.calculate(x, targetPose.getX());
        double YOutput = 0.15 * kLineupSpeed.in(MetersPerSecond) * 0.95 * xDistanceController.calculate(y, targetPose.getY());
        double thetaOutput = 0.15 * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        if (DriverStation.getAlliance().get() == Alliance.Blue){
            CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
                RobotContainer.drive.withVelocityX(-XOutput)
                     .withVelocityY(-YOutput)
                     .withRotationalRate(thetaOutput)
                     
            ).execute();
        } else {
            CommandSwerveDrivetrain.getInstance().applyRequest(() -> 
            RobotContainer.drive.withVelocityX(XOutput)
                     .withVelocityY(YOutput)
                     .withRotationalRate(-thetaOutput)   
            ).execute();
        }

        SmartDashboard.putString("Target Pose X", targetPose.toString());

    }

    @Override
    public boolean isFinished() {
        return xDistanceController.atSetpoint() && yDistanceController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    private Pose2d getTargetReefPose(boolean left){
        var photonResult = leftCam.getAllUnreadResults();
        var currAlliance = DriverStation.getAlliance();
        
        for (var result : photonResult){
            if (result.hasTargets()){
                tagID = result.getBestTarget().fiducialId;
            }
        }

        if (currAlliance.get() == Alliance.Blue){

            if (tagID == 17){
                if (left) {
                    return Cblue;
                } else if (!left) {
                    return Dblue;
                }
            } else if (tagID == 18) {
                if (left) {
                    return Ablue;
                } else if (!left) {
                    return Bblue;
                }
            } else if (tagID == 19) {
                if (left) {
                    return Kblue;
                } else if (!left) {
                    return Lblue;
                }
            } else if (tagID == 20) {
                if (left) {
                    return Iblue;
                } else if (!left) {
                    return Jblue;
                }
            } else if (tagID == 21) {
                if (left) {
                    return Gblue;
                } else if (!left) {
                    return Hblue;
                }
            } else if (tagID == 22) {
                if (left) {
                    return Eblue;
                } else if (!left) {
                    return Fblue;
                }
            }

        } else if (currAlliance.get() == Alliance.Red){
            if (tagID == 7){
                if (left) {
                    return Ared;
                } else if (!left) {
                    return Bred;
                }
            } else if (tagID == 8) {
                if (left) {
                    return Cred;
                } else if (!left) {
                    return Dred;
                }
            } else if (tagID == 9) {
                if (left) {
                    return Ered;
                } else if (!left) {
                    return Fred;
                }
            } else if (tagID == 10) {
                if (left) {
                    return Gred;
                } else if (!left) {
                    return Hred;
                }
            } else if (tagID == 11) {
                if (left) {
                    return Ired;
                } else if (!left) {
                    return Jred;
                }
            } else if (tagID == 6) {
                if (left) {
                    return Kred;
                } else if (!left) {
                    return Lred;
                }
            }

        } else if (currAlliance.get() != Alliance.Red && currAlliance.get() != Alliance.Blue){
            System.out.println("ERROR: There's no such thing as purple alliance");
        }

        System.out.println("Error: no suitable target pose found (Lineup Command)");
        return currentPose;

    }


}
