package virtual_robot.ftcfield;

import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBodyHelper;
import org.ode4j.ode.*;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

import java.util.List;

import static org.ode4j.ode.OdeConstants.*;

/**
 * Minimal placeholder field: just a floor and four boundary walls, no game pieces.
 * Stand-in until real field art/geometry for the current game is added.
 */
public class DarbotsField extends FtcField {

    private static DarbotsField instance = null;

    private final double HALF_FIELD_WIDTH;
    private final Image backgroundImage = Config.BACKGROUND;

    public static DarbotsField getInstance(Group group, DWorld world, DSpace space) {
        if (instance == null) {
            instance = new DarbotsField(group, world, space);
        }
        return instance;
    }

    private DarbotsField(Group group, DWorld world, DSpace space) {
        super(group, world, space);
        HALF_FIELD_WIDTH = VirtualRobotController.HALF_FIELD_WIDTH;
    }

    @Override
    public void setup() {
        TriangleMesh fieldMesh = Util3D.getParametricMesh(-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH, -HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,
                10, 10, false, false, new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) { return s; }

                    @Override
                    public float y(float s, float t) { return t; }

                    @Override
                    public float z(float s, float t) { return 0; }
                });
        MeshView fieldView = new MeshView(fieldMesh);
        PhongMaterial fieldMaterial = new PhongMaterial();
        fieldMaterial.setDiffuseColor(Color.gray(0.02));
        fieldMaterial.setDiffuseMap(backgroundImage);
        fieldMaterial.setSelfIlluminationMap(backgroundImage);
        fieldView.setMaterial(fieldMaterial);
        subSceneGroup.getChildren().add(fieldView);

        DPlane fieldPlane = OdeHelper.createPlane(space, 0, 0, 1, 0);
        fieldPlane.setData("Field Plane");
        fieldPlane.setCategoryBits(CBits.FLOOR);

        PhongMaterial wallMaterial = new PhongMaterial(Color.color(0.02, 0.02, 0.02, 0));
        for (int i = 0; i < 4; i++) {
            TriangleMesh wallMesh = Util3D.getParametricMesh(-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH, -15, 15,
                    10, 10, false, false, new Util3D.Param3DEqn() {
                        @Override
                        public float x(float s, float t) { return s; }

                        @Override
                        public float y(float s, float t) { return 0; }

                        @Override
                        public float z(float s, float t) { return t; }
                    });
            MeshView wallView = new MeshView(wallMesh);
            wallView.setMaterial(wallMaterial);
            double dx = i == 0 ? 0 : i == 1 ? -HALF_FIELD_WIDTH - 2 : i == 2 ? 0 : HALF_FIELD_WIDTH + 2;
            double dy = i == 0 ? HALF_FIELD_WIDTH + 2 : i == 1 ? 0 : i == 2 ? -HALF_FIELD_WIDTH - 2 : 0;
            double angle = i == 0 ? 0 : i == 1 ? 90 : i == 2 ? 180 : -90;
            wallView.getTransforms().addAll(new Translate(dx, dy, 15), new Rotate(angle, Rotate.Z_AXIS));
            subSceneGroup.getChildren().add(wallView);
            List<DGeom> wallGeoms = FxBodyHelper.dGeomsFromNode(wallView, space, null);
            for (DGeom dg : wallGeoms) dg.setCategoryBits(CBits.WALLS);
        }
    }

    @Override
    public void reset() {
        // No dynamic game pieces to reset.
    }

    @Override
    public void updateDisplay() {
        // No dynamic game pieces to redraw.
    }

    @Override
    public void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup) {
        for (int i = 0; i < numContacts; i++) {
            DContact contact = contacts.get(i);
            contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
            contact.surface.bounce = 0.3;
            contact.surface.bounce_vel = 2.0;
            contact.surface.mu = 0.5;
            contact.surface.soft_cfm = 0;
            contact.surface.soft_erp = 0.4;
            DJoint c = OdeHelper.createContactJoint(world, contactGroup, contact);
            c.attach(contact.geom.g1.getBody(), contact.geom.g2.getBody());
        }
    }
}
