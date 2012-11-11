package ex3.pid;

import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/*
 * Adapted from http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
public class PID extends AbstractNodeMain {

    public static enum DIRECTION {DIRECT, REVERSE};

    double proportionalGain, integralGain, derivativeGain;
    double input; // The measured value of the variable that we are trying to control
    double setpoint; // The actual value that we want the variable to be.
    double output = 0;

    double lastTime; // Time elapsed since the last call to the controller.
    double integralTerm, lastError;
    double lastInput; // We use this to fix the derivative kick phenomenon

    double max, min;

    boolean active = false;
    DIRECTION pidDirection;

    Subscriber<PoseWithCovarianceStamped> localisationPose;
    Publisher<Quaternion> controlPub;
    public static MessageFactory factory;

    public PID(double proportionalGain, double integralGain, double derivativeGain, double min, double max, DIRECTION direction){
        pidDirection = direction;
        System.out.println("pid constructed");
        if (pidDirection == DIRECTION.DIRECT){
            this.proportionalGain = proportionalGain;
            this.integralGain = integralGain;
            this.derivativeGain = derivativeGain;
        } else {
            // If this is a reverse process, e.g. an increase of our output
            // decreases the value of the input, then we reverse the gain params
            this.proportionalGain = -proportionalGain;
            this.integralGain = -integralGain;
            this.derivativeGain = -derivativeGain;
        }
        setOutputLimits(min, max);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

        System.out.println("onstart called");
        localisationPose = connectedNode.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);

        localisationPose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
            @Override
            public void onNewMessage(PoseWithCovarianceStamped t) {
                // We should be called every time we receive a message from the
                // localisation node.
                System.out.println("pid got message");
                input = getHeading(t.getPose().getPose().getOrientation());
                control();
            }
        });

        controlPub = connectedNode.newPublisher("PID_Rotation", Quaternion._TYPE);

        factory = connectedNode.getTopicMessageFactory(); // To create quaternions
    }

    public void control(){
        if (!active) return; // Don't modify any values if we're not running.

        double timeNow = System.currentTimeMillis();
        // Time delta between now and the last time we called the controller.
        double deltaT = timeNow - lastTime;

        System.out.println("deltaT = "+ deltaT);

        // Proportional error - How different is the set point compared to the
        // value that we are getting out of the sensor.
        double error = setpoint - input;
        System.out.println("prop error: " + error);

        // The total error up until now. Multiply by error to compensate for
        // bump caused by the modification of parameters during operation.
        integralTerm += setToLimits(integralGain * error * deltaT);
        System.out.println("integral term: " + integralTerm);

        double deltaIn = lastInput - input;
        System.out.println("change in input from last call: " + deltaIn);

        // How fast the error is changing.
        double derivativeError = (error - lastError) / deltaT;
        System.out.println("deriv error: " + derivativeError);

        // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
        output = setToLimits(proportionalGain * error + integralTerm - derivativeGain * (deltaIn / deltaT));

        System.out.println("output is: " + output);

        lastInput = input;
        lastError = error;
        lastTime = timeNow;

    }

    public void setOutputLimits(double min, double max){
        if (min > max){
            throw new IllegalStateException("min cannot be greater than max");
        }

        this.min = min;
        this.max = max;

        setToLimits(output);
        setToLimits(integralTerm);
    }

    /*
     * Ensures that a variable is within the set limits
     */
    public double setToLimits(double value){
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        }
        
        return value;
    }

    /*
      * Activates the controller.
      */
    public void activate(){
        // If we switch states from inactive to active, set values to avoid bump.
        if (active = false){
            lastInput = input;
            integralTerm = setToLimits(output);
        }
        active = true;
    }

    /*
     * Deactivates the controller.
     */
    public void deactivate(){
        active = false;
    }

    public boolean getState(){
        return active;
    }

    public double getOutput() {
        return output;
    }
    
    public void setSetpoint(double setpoint) {
        System.out.println("Setting setpoint to " + setpoint);
        this.setpoint = setpoint;
    }

    public double getDerivativeGain() {
        return derivativeGain;
    }

    public double getIntegralGain() {
        return integralGain;
    }

    public double getProportionalGain() {
        return proportionalGain;
    }

    public void setDerivativeGain(double derivativeGain) {
        this.derivativeGain = derivativeGain;
    }

    public void setIntegralGain(double integralGain) {
        this.integralGain = integralGain;
    }

    public void setProportionalGain(double proportionalGain) {
        this.proportionalGain = proportionalGain;
    }

    /** Quaternion code from Mark Rowan's implementation of localisation **/

    /**
     * Converts a basic rotation about the z-axis (in radians) into the Quaternion
     * notation required by ROS transform and pose messages.
     * @param yaw rotate by this amount in radians
     * @param q Quaternion to be rotated
     * @return quaternion rotation about the z-axis (fine for robots which only
     * rotate using two-wheeled differential drive, for example)
     */
    public static Quaternion rotateQuaternion(Quaternion q_orig, double yaw) {
        // Create a temporary Quaternion to represent the change in heading
        Quaternion q_headingChange = createQuaternion();

        double p = 0;
        double y = yaw / 2.0;
        double r = 0;

        double sinp = Math.sin(p);
        double siny = Math.sin(y);
        double sinr = Math.sin(r);
        double cosp = Math.cos(p);
        double cosy = Math.cos(y);
        double cosr = Math.cos(r);

        q_headingChange.setX(sinr * cosp * cosy - cosr * sinp * siny);
        q_headingChange.setY(cosr * sinp * cosy + sinr * cosp * siny);
        q_headingChange.setZ(cosr * cosp * siny - sinr * sinp * cosy);
        q_headingChange.setW(cosr * cosp * cosy + sinr * sinp * siny);

        // Multiply new (heading-only) quaternion by the existing (pitch and bank) quaternion
        // Order is important! Original orientation is the second argument;
        // rotation which will be applied to the quaternion is the first argument.
        return multiply_quaternions(q_headingChange, q_orig);
    }


    /**
     * Multiplies two quaternions to give the rotation of qb by qa.
     * @param qa Quaternion rotation amount which will be applied to qb.
     * @param qb Quaternion to which rotation of qa will be applied.
     * @return Quaternion qb rotated by the amount represented by qa.
     */
    private static Quaternion multiply_quaternions( Quaternion qa, Quaternion qb ) {
        Quaternion combined = createQuaternion();

        combined.setW(qa.getW()*qb.getW() - qa.getX()*qb.getX() - qa.getY()*qb.getY() - qa.getZ()*qb.getZ());
        combined.setX(qa.getX()*qb.getW() + qa.getW()*qb.getX() + qa.getY()*qb.getZ() - qa.getZ()*qb.getY());
        combined.setY(qa.getW()*qb.getY() - qa.getX()*qb.getZ() + qa.getY()*qb.getW() + qa.getZ()*qb.getX());
        combined.setZ(qa.getW()*qb.getZ() + qa.getX()*qb.getY() - qa.getY()*qb.getX() + qa.getZ()*qb.getW());
        return combined;

    }


    /**
     *
     * @return A blank identity quaternion (x,y,z = 0, w = 1) representing zero rotation.
     */
    public static Quaternion createQuaternion() {
        Quaternion q = factory.newFromType(Quaternion._TYPE);

        // Set up 'identity' (blank) quaternion
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);
        return q;
    }

    /**
     *
     * @param q Quaternion describing a particular orientation about the z-axis
     * @return Equivalent orientation about the z-axis in radians
     */
    public static double getHeading(Quaternion q) {
        double w = q.getW();
        double x = q.getX();
        double y = q.getY();
        double z = q.getZ();

//        double pitch = Math.atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
//        double roll = Math.asin(-2*(x*z - w*y));
        double yaw = Math.atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
        return yaw;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("PID");
    }
}
