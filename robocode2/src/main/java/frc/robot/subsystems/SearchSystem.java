package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;

public class SearchSystem extends SubsystemBase {
    NetworkTableInstance ntinst;
    NetworkTable powerCellVision;
    NetworkTableEntry targetArea, targetWidth;

    public SearchSystem() {
        ntinst = NetworkTableInstance.getDefault();
        powerCellVision = ntinst.getTable("power_cell_vision");
        targetArea = powerCellVision.getEntry("target_area");
        targetWidth = powerCellVision.getEntry("target_width");
    }
}