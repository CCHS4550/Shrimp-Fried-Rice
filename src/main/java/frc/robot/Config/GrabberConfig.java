package frc.robot.Config;

public class GrabberConfig {
    public double grabberKp;
    public double grabberKi;
    public double grabberKd;
    public double grabberKv;
    public double grabberKs;

    public GrabberConfig withGrabberKp(double grabberKp){
        this.grabberKp = grabberKp;
        return this;
    }
    public GrabberConfig withGrabberKi(double grabberKi){
        this.grabberKi = grabberKi;
        return this;
    }
    public GrabberConfig withGrabberKd(double grabberKd){
        this.grabberKd = grabberKd;
        return this;
    }
    public GrabberConfig withGrabberKv(double grabberKv){
        this.grabberKv = grabberKv;
        return this;
    }
    public GrabberConfig withGrabberKs(double grabberKs){
        this.grabberKs = grabberKs;
        return this;
    }
}
