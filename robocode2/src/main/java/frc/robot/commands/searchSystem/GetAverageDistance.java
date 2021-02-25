package frc.robot.commands.searchSystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SearchSystem;

public class GetAverageDistance extends CommandBase {
    private Timer timer;
    private SearchSystem searchSystem;
    private double time, average;

    public GetAverageDistance(SearchSystem searchSystem, double time) {
        this.time = time;
        this.searchSystem = searchSystem;

        timer = new Timer();
    }

    public void initialize() {
        timer.start();
        average = searchSystem.estimateDistance();
    }

    public void execute() {
        average = (average + searchSystem.estimateDistance()) /2;
        searchSystem.averageEstimatedDisatnce = average;
    }

    public void end() {
        searchSystem.averageEstimatedDisatnce = average;
    }

    public boolean isFinished() {
        if(timer.get() >= time) {
            return true;
        }
        return false;
    }
}
