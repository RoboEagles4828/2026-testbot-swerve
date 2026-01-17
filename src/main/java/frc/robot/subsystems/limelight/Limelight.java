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

    public Limelight() {

    }

    @Override
    public void periodic() {
        // feed the robot's current rotation to the limelight (required for MegaTag2 algorithm)
        if (drivetrain != null) {
            LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_NAME, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        }

        // get the latest pose estimate (using MegaTag2) from the drivetrain and check its quality
        LimelightHelpers.PoseEstimate poseEstimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LIMELIGHT_NAME);
        LimelightHelpers.PoseEstimate poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_NAME);
        
        if (poseEstimateMT1 != null) {
            Util4828.FIELD.getObject("MT1").setPose(poseEstimateMT1.pose);
        }

        if (poseEstimateMT2 != null) {
            Util4828.FIELD.getObject("MT2").setPose(poseEstimateMT2.pose);
        }

        SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber("LL TY", LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber("LL Tag ID", LimelightHelpers.getFiducialID(LimelightConstants.LIMELIGHT_NAME));
        SmartDashboard.putString("LL Pose (MT2)", poseEstimateMT2 == null ? "NULL" : Util4828.formatPose(poseEstimateMT2.pose));
        SmartDashboard.putString("LL Pose (MT1)", poseEstimateMT1 == null ? "NULL" : Util4828.formatPose(poseEstimateMT1.pose));
        SmartDashboard.putNumber("LL Timestamp", poseEstimateMT2 == null ? -1 : poseEstimateMT2.timestampSeconds);
    }
}
