// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.WaitSmartDashBoard;
import frc.robot.commands.drivetrain.DriveSwerveCommand;
import frc.robot.commands.limelight.SetMegaTag;
import frc.robot.commands.limelight.SetMegaTag2;
import frc.robot.commands.limelight.SetNoCamera;
import frc.robot.commands.winch.RollWinchSpeed;
import frc.robot.commands.winch.RollWinchStick;
import frc.robot.generated.TunerConstants;
import frc.robot.limelight.Limelight2025;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.LimelightToUse;
import frc.robot.subsystems.Winch;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_helperController = new CommandXboxController(OperatorConstants.kHelperControllerPort);

    // Subsystems

    public final Limelight2025 limelightThree = new Limelight2025("limelight-three", LimelightToUse.LIMELIGHT_THREE, this);
    public final Limelight2025 limelightFour = new Limelight2025("limelight-four", LimelightToUse.LIMELIGHT_FOUR, this);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(limelightFour, limelightThree);
    public final Winch m_winch = new Winch();
    public final SmartDashboardSettings smartDashboardSettings = new SmartDashboardSettings();

    private final DriveSwerveCommand driveSwerveCommand = new DriveSwerveCommand(drivetrain, m_driverController, limelightFour);

    private final RollWinchStick m_rollWinchStick = new RollWinchStick(m_winch, m_helperController);
    private final RollWinchSpeed m_stopWinch = new RollWinchSpeed(m_winch, 0.0);


    public Alliance m_alliance = Alliance.Red;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public double auto_delay = 0.0;

    public RobotContainer() {
        autoChooser = buildAutoChooser();
        configureBindings();
        configureDefaultCommands();
    }

    private SendableChooser<Command> buildAutoChooser() {
        final SendableChooser<Command> autoChooser;
        NamedCommands.registerCommand("Auto Delay", new WaitSmartDashBoard(smartDashboardSettings));
        NamedCommands.registerCommand("Set Mega Tag", new SetMegaTag(limelightFour, drivetrain));
        NamedCommands.registerCommand("Set Mega Tag 2", new SetMegaTag2(limelightFour, drivetrain));
        NamedCommands.registerCommand("Set No Camera", new SetNoCamera(limelightFour, drivetrain));
        autoChooser = AutoBuilder.buildAutoChooser("auto_path");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putNumber("Auto Delay", auto_delay);
        return autoChooser;
    }

    private void configureBindings() {
        // reset the field-centric heading on left bumper press
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_helperController.rightStick().whileTrue(m_rollWinchStick);
    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(driveSwerveCommand);
        m_winch.setDefaultCommand(m_stopWinch);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void setAlliance() {
        m_alliance = DriverStation.getAlliance().orElse(m_alliance);
    }

    public void teleopInit() {
        setAlliance();
    }

    public void autonomousInit() {
        setAlliance();
    }

    public void robotPeriodic() {
        SmartDashboardWrapper.putNumber("Match Time", DriverStation.getMatchTime());
    }
}
