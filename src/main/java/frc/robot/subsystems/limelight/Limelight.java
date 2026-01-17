package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Util4828;

public class Limelight extends SubsystemBase {
    
    private CommandSwerveDrivetrain drivetrain;

    public Limelight(CommandSwerveDrivetrain drive) {
        drivetrain = drive;
    }

    @Override
    public void periodic() {
        // feed the robot's current rotation to the limelight (required for MegaTag2 algorithm)
        LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_NAME, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
        // get the latest pose estimate (using MegaTag2) from the drivetrain and check its quality
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_NAME);

        SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber("LL TY", LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber("LL Tag ID", LimelightHelpers.getFiducialID(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putString("LL Pose (MT2)", poseEstimate == null ? "NULL" : Util4828.formatPose(poseEstimate.pose));
        SmartDashboard.putNumber("LL Timestamp", poseEstimate == null ? -1 : poseEstimate.timestampSeconds);
    }
}
