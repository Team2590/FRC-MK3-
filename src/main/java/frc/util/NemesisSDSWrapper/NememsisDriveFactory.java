package frc.util.NemesisSDSWrapper;

import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface NememsisDriveFactory<Controller extends DriveController, DriveConfiguration> {
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller
    ) {
        container.addNumber("Current Velocity", controller::getStateVelocity);
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            ModuleConfiguration moduleConfiguration
    ) {
        var controller = create(driveConfiguration, moduleConfiguration);
        addDashboardEntries(container, controller);

        return controller;
    }

    Controller create(DriveConfiguration driveConfiguration, ModuleConfiguration moduleConfiguration);
}