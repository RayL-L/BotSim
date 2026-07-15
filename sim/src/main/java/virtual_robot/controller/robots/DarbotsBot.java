package virtual_robot.controller.robots;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.Rotate;

import org.darbots.config.RobotHardwareConfig;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;

/**
 * Bot config for the actual Darbots 5100 robot: a plain box chassis on 4 mecanum wheels,
 * plus shootL/shootR/Intake/Ramp drive-map entries for the Shooters subsystem. Drive
 * kinematics/physics follow the same kinematic-model-to-force approach as UltimateBot.java
 * (the only other complete bot config in this project) -- real ODE4J physics, just without
 * UltimateBot's game-specific shooter/grabber mechanism geometry.
 *
 * Chassis dimensions/mass below are placeholder estimates for a generic 18"x18" FTC robot --
 * tune to match the real robot's actual dimensions/weight as they become known.
 */
@BotConfig(name = "Darbots 5100")
public class DarbotsBot extends VirtualBot {

    private final float TOTAL_MASS = 15000; // mg, placeholder -- tune to real robot weight
    private final float TOTAL_Z_INERTIA = 5000000f; // gm*cm2, placeholder
    private final float FIELD_FRICTION_COEFF = 1.0f;
    private final float GRAVITY = 980f; // cm/s2
    private final float MAX_WHEEL_X_FORCE = TOTAL_MASS * GRAVITY * FIELD_FRICTION_COEFF / (4.0f * (float) Math.sqrt(2));

    private final MotorType motorType = MotorType.Neverest40;

    private DcMotorExImpl[] motors = null; // LF, RF, LB, RB (index order below)
//    private DcMotorExImpl shootLMotor = null;
//    private DcMotorExImpl shootRMotor = null;
//    private DcMotorExImpl intakeMotor = null;
//    private DcMotorExImpl rampMotor = null;
    private BNO055IMUImpl imu = null;

    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    private double[][] tWR; // wheel motion -> robot motion

    private GeneralMatrixF M_ForceWheelToRobot;
    private MatrixF M_ForceRobotToWheel;

    private final Rotate[] wheelRotates = new Rotate[]{
            new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS),
            new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS)
    };
    private final double[] wheelRotations = new double[]{0, 0, 0, 0};

    @Override
    public void init() {
        super.init();
        hardwareMap.setActive(true);

        // Order matches UltimateBot's convention: back_left, front_left, front_right, back_right
        motors = new DcMotorExImpl[]{
                hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.LB),
                hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.LF),
                hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.RF),
                hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.RB)
        };
//        shootLMotor = hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.SHOOT_L);
//        shootRMotor = hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.SHOOT_R);
//        intakeMotor = hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.INTAKE);
//        rampMotor = hardwareMap.get(DcMotorExImpl.class, RobotHardwareConfig.RAMP);
        imu = hardwareMap.get(BNO055IMUImpl.class, RobotHardwareConfig.IMU);

        hardwareMap.setActive(false);

        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;

        tWR = new double[][]{
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25 / wlAverage, -0.25 / wlAverage, 0.25 / wlAverage, 0.25 / wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };

        float RRt2 = 0.5f * (float) Math.sqrt(interWheelLength * interWheelLength + interWheelWidth * interWheelWidth) * (float) Math.sqrt(2.0);
        M_ForceWheelToRobot = new GeneralMatrixF(4, 4, new float[]{
                1, 1, 1, 1,
                -1, 1, -1, 1,
                RRt2, -RRt2, -RRt2, RRt2,
                1, 1, -1, -1});
        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();
    }

    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();
        String[] driveMotorNames = new String[]{
                RobotHardwareConfig.LB, RobotHardwareConfig.LF, RobotHardwareConfig.RF, RobotHardwareConfig.RB
        };
        for (String name : driveMotorNames) hardwareMap.put(name, new DcMotorExImpl(motorType));

        hardwareMap.put(RobotHardwareConfig.SHOOT_L, new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put(RobotHardwareConfig.SHOOT_R, new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put(RobotHardwareConfig.INTAKE, new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put(RobotHardwareConfig.RAMP, new DcMotorExImpl(MotorType.Neverest40));

        hardwareMap.put(RobotHardwareConfig.IMU, new BNO055IMUImpl(this, 10));
    }

    public synchronized void updateSensors() {
        DMatrix3C currentRot = fxBody.getRotation();
        double headingRadians = Math.atan2(currentRot.get10(), currentRot.get00());
        imu.updateHeadingRadians(headingRadians);
    }

    public synchronized void updateState(double millis) {
        double[] deltaTicks = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaTicks[i] = motors[i].update(millis);
            w[i] = deltaTicks[i] * wheelCircumference / motorType.TICKS_PER_ROTATION;
            double wheelRotationDegrees = 360.0 * deltaTicks[i] / motorType.TICKS_PER_ROTATION;
            if (i < 2) {
                w[i] = -w[i];
                wheelRotationDegrees = -wheelRotationDegrees;
            }
            wheelRotations[i] += Math.min(17, Math.max(-17, wheelRotationDegrees));
        }

        double[] robotDeltaPos = new double[]{0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double dHeading = robotDeltaPos[2];

        DMatrix3C currentRot = fxBody.getRotation();
        double heading = Math.atan2(currentRot.get10(), currentRot.get00());
        double avgHeading = heading + dHeading / 2.0;
        double sinAvg = Math.sin(avgHeading);
        double cosAvg = Math.cos(avgHeading);
        double dX = dxR * cosAvg - dyR * sinAvg;
        double dY = dxR * sinAvg + dyR * cosAvg;

        double t = millis / 1000.0;
        double tSqr = t * t;
        DVector3 deltaPos = new DVector3(dX, dY, 0);
        DVector3C vel = fxBody.getLinearVel().clone();
        ((DVector3) vel).set2(0);
        DVector3 force = deltaPos.reSub(vel.reScale(t)).reScale(2.0 * TOTAL_MASS / tSqr);
        double angVel = fxBody.getAngularVel().get2();
        float torque = (float) (2.0 * TOTAL_Z_INERTIA * (dHeading - angVel * t) / tSqr);

        double sinHd = Math.sin(heading);
        double cosHd = Math.cos(heading);
        float fXR = (float) (force.get0() * cosHd + force.get1() * sinHd);
        float fYR = (float) (-force.get0() * sinHd + force.get1() * cosHd);

        VectorF totalBotForces = new VectorF(fXR, fYR, torque, 0);
        VectorF wheel_X_Forces = M_ForceRobotToWheel.multiplied(totalBotForces);

        for (int i = 0; i < 4; i++) {
            float f = wheel_X_Forces.get(i);
            if (Math.abs(f) > MAX_WHEEL_X_FORCE) {
                wheel_X_Forces.put(i, MAX_WHEEL_X_FORCE * Math.signum(f));
            }
        }

        VectorF frictionForces = M_ForceWheelToRobot.multiplied(wheel_X_Forces);
        force.set0(frictionForces.get(0) * cosHd - frictionForces.get(1) * sinHd);
        force.set1(frictionForces.get(0) * sinHd + frictionForces.get(1) * cosHd);
        force.set2(0);

        fxBody.addForce(force);
        fxBody.addTorque(new DVector3(0, 0, frictionForces.get(2)));

//        shootLMotor.update(millis);
//        shootRMotor.update(millis);
//        intakeMotor.update(millis);
//        rampMotor.update(millis);
    }

    protected void setUpFxBody() {
        DWorld world = controller.getWorld();
        fxBody = odefx.FxBody.newInstance(world, botSpace);
        DBody chassisBody = fxBody;
        DMass chassisMass = OdeHelper.createMass();
        chassisMass.setMass(TOTAL_MASS);
        chassisMass.setI(new DMatrix3(TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA));
        chassisBody.setMass(chassisMass);

        double chassisSide = botWidth * 0.9;
        double chassisHeight = botWidth * 0.15;

        Group botGroup = new Group();
        Box chassis = new Box(chassisSide, chassisSide, chassisHeight);
        PhongMaterial chassisMaterial = new PhongMaterial(Color.DARKSLATEGRAY);
        chassisMaterial.setSpecularColor(Color.WHITE);
        chassis.setMaterial(chassisMaterial);
        botGroup.getChildren().add(chassis);

        double wheelDiam = botWidth * 0.22;
        double wheelWidth = botWidth * 0.12;
        double wheelXOffset = chassisSide / 2.0 - wheelWidth / 2.0;
        double wheelYOffset = chassisSide / 2.0 - wheelDiam / 2.0;

        PhongMaterial wheelMaterial = new PhongMaterial(Color.BLACK);
        wheelMaterial.setSpecularColor(Color.WHITE);
        for (int i = 0; i < 4; i++) {
            Cylinder wheel = new Cylinder(wheelDiam / 2.0, wheelWidth);
            wheel.setMaterial(wheelMaterial);
            wheel.setRotationAxis(new Point3D(0, 0, 1));
            wheel.setRotate(90);
            wheel.setTranslateX(i < 2 ? -wheelXOffset : wheelXOffset);
            wheel.setTranslateY(i == 0 || i == 3 ? -wheelYOffset : wheelYOffset);
            wheel.getTransforms().add(wheelRotates[i]);
            botGroup.getChildren().add(wheel);
        }

        fxBody.setNode(botGroup, false);

        DBox chassisBox = OdeHelper.createBox(chassisSide, chassisSide, chassisHeight);
        fxBody.addGeom(chassisBox, 0, 0, 0);

        zBase = chassisHeight / 2.0 + wheelDiam / 2.0 - chassisHeight / 2.0 + 1.0;

        fxBody.setCategoryBits(odefx.CBits.BOT);
        fxBody.setCollideBits(0xFFFFFFFFFFFFFFFFL);
    }

    public synchronized void updateDisplay() {
        super.updateDisplay();
        for (int i = 0; i < 4; i++) wheelRotates[i].setAngle(wheelRotations[i]);
    }

    public void powerDownAndReset() {
        for (int i = 0; i < 4; i++) motors[i].stopAndReset();
//        shootLMotor.stopAndReset();
//        shootRMotor.stopAndReset();
//        intakeMotor.stopAndReset();
//        rampMotor.stopAndReset();
        imu.close();
    }

    public synchronized void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup) {
        for (int i = 0; i < numContacts; i++) {
            DContact contact = contacts.get(i);
            contact.surface.mode = OdeConstants.dContactSoftERP | OdeConstants.dContactSoftCFM | OdeConstants.dContactApprox1 | OdeConstants.dContactBounce;
            contact.surface.mu = 0.5;
            contact.surface.soft_cfm = 0.00000001;
            contact.surface.soft_erp = 0.2;
            contact.surface.bounce = 0.3;
            contact.surface.bounce_vel = 10;
            DJoint c = OdeHelper.createContactJoint(controller.getWorld(), contactGroup, contact);
            c.attach(contact.geom.g1.getBody(), contact.geom.g2.getBody());
        }
    }
}
