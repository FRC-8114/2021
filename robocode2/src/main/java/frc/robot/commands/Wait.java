package frc.robot.commands;

import javax.naming.InitialContext;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Thread;


public class Wait extends CommandBase {
    long time;
    
    public Wait(long time) {
        this.time = time;
    }
    
    public void initialize() {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
