package frc.robot.Auto;

import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;


public class boxandturn extends SequentialCommandGroup {
    public boxandturn(RobotContainer robot) {
    
                PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("New Path", new PathConstraints(4,3));
                PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("New New Path", new PathConstraints(2,1.75));
                
                var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController,0, 0,Constants.AutoConstants.kThetaControllerConstraints);
                
                thetaController.enableContinuousInput(0, 2 * Math.PI);
                
                        PPSwerveControllerCommand swerveControllerCommand1 =
                        new PPSwerveControllerCommand(
                    trajectory2,
                    robot.s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPThetaController, 0.005, 0),
                    robot.s_Swerve::setModuleStates,
                    robot.s_Swerve);

                    PPSwerveControllerCommand swerveControllerCommand2 =
                    new PPSwerveControllerCommand(
                trajectory3,
                robot.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(Constants.AutoConstants.kPThetaController, .005, 0),
                robot.s_Swerve::setModuleStates,
                robot.s_Swerve);
                
                    addCommands(
                        new InstantCommand(
                            () ->
                                robot.s_Swerve.resetOdometry(
                                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)))),
                    
                                    swerveControllerCommand1
                                    //new WaitCommand(2),
                                   // swerveControllerCommand2
                                    );
            
}
}
