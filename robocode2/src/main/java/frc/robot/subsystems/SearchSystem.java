package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;

public class SearchSystem extends SubsystemBase {
    private NetworkTableInstance ntinst;
    public NetworkTable powerCellVision;
    private NetworkTableEntry targetArea, targetWidth, centerX;
    private double targetCenterX;

    public SearchSystem() {
        ntinst = NetworkTableInstance.getDefault();
        powerCellVision = ntinst.getTable("power_cell_vision");
        targetArea = powerCellVision.getEntry("target_area");
        targetWidth = powerCellVision.getEntry("target_width");
        centerX =  powerCellVision.getEntry("target_centerX");
        targetCenterX = centerX.getDouble(0.0);
    }

    /**
     * Estimates the target's distance by averaging bot the area and
     * width methods
     * 
     * @return the estimated distance to target
     */
    public double estimateDistance() {
        return (widthEstimateDistance());
    }

    /**
     * Estimates the target's distance based off of a function derived
     * from experimental datapoints and its area
     * 
     * @return the estimated distance to target
     */
    private double areaEstimateDistance() {
        return 253 * Math.pow(targetArea.getDouble(0.0), -0.353);
    }


    /**
     * Estimates the target's distance based off of a function derived
     * from experimental datapoints and its width
     * 
     * @return the estimated distance to target
     */
    private double widthEstimateDistance() {
        return 559 * Math.pow(targetWidth.getDouble(0.0), -0.828);
    }

    
    /**
     * Sends the estimated distance to the power_cell_vision
     * networktable
     */
    public void sendEstimatedDistance() {
        powerCellVision.getEntry("estimatedDistance").forceSetDouble(estimateDistance());
        
        powerCellVision.getEntry("areaDistance").forceSetDouble(areaEstimateDistance());
        
        powerCellVision.getEntry("widthDistance").forceSetDouble(widthEstimateDistance());

        //powerCellVision.getEntry("averageEstimatedDistance").forceSetDouble(averageDistance());

        powerCellVision.getEntry("centerX").forceSetDouble(centerX.getDouble(0.0));

        powerCellVision.getEntry("determinedPath").forceSetString(pathDetermination());
    }

    public String pathDetermination() {
        double averageDistance = powerCellVision.getEntry("averageEstimatedDisatnce").getDouble(0.0);

        if((averageDistance) <= 110) {
            return "redPath";
        }
        else if( (averageDistance >= 110) && (averageDistance <= 190) ) {
            if(targetCenterX < 80) {
                return "blueVert";
            }
            else
                return "blueHor";
        }

        return "";
    }

}