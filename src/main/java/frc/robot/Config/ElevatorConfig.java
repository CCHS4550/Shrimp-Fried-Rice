package frc.robot.Config;

public class ElevatorConfig {
    public double elevatorKp;
    public double elevatorKi;
    public double elevatorKd;
    public double elevatorKv;
    public double elevatorKs;

    public ElevatorConfig withElevatorKp(double elevatorKp){
        this.elevatorKp = elevatorKp;
        return this;
    }
    public ElevatorConfig withGrabberKi(double elevatorKi){
        this.elevatorKi = elevatorKi;
        return this;
    }
    public ElevatorConfig withGrabberKd(double elevatorKd){
        this.elevatorKd = elevatorKd;
        return this;
    }
    public ElevatorConfig withGrabberKv(double elevatorKv){
        this.elevatorKv = elevatorKv;
        return this;
    }
    public ElevatorConfig withGrabberKs(double elevatorKs){
        this.elevatorKs = elevatorKs;
        return this;
    }
}
