package virtual_robot.ftcfield;

import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.*;
import util3d.ObjMeshLoader;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

import java.util.ArrayList;
import java.util.List;

import static org.ode4j.ode.OdeConstants.*;

/**
 * 2026-2027 BIOBUZZ field. Structural props (Frame, Flowers, Hives) are the real Onshape field CAD,
 * decimated to low-poly OBJs and loaded from resources; game pieces (Pollen, Nectar) are physics
 * primitives. This is the CAD-visual stage of the hybrid plan: real geometry for looks, primitive
 * ODE geoms for collision.
 *
 * Meshes are already Z-up / centered / at their true field positions (units converted m->cm by the
 * offline filter script), so they load at the identity transform. Collision is approximated:
 *   - Frame: a central box proxy (the A-frame's open legs are not individually modelled)
 *   - Flowers: none (so staged Pollen stay grabbable by a base-only bot)
 *   - Hives: none (they sit ~1 m up, out of a drivetrain-only robot's reach)
 *
 * Deferred (documented): the Hive is static here -- restoring the tipping mechanic needs the hive
 * mesh split per-alliance and centered on its pivot, then re-hinged. Per-alliance red/blue hive
 * tint also needs that split (one mesh currently uses a single neutral material).
 */
public class BioBuzzField extends FtcField {

    private static final String MESH_DIR = "/virtual_robot/ftcfield/meshes/";

    // ---- Field grid (units: centimeters / grams) -------------------------------------------
    private static final double TILE = 60.96;              // 24 in. foam tile

    // ---- Game pieces -----------------------------------------------------------------------
    private static final double POLLEN_R    = 3.81;        // 3 in. ball, radius
    private static final double POLLEN_MASS = 40.0;
    private static final double NECTAR_R    = 3.0;
    private static final double NECTAR_LEN  = 6.0;
    private static final double NECTAR_MASS = 50.0;

    // Real flower centers (pinwheel arrangement, from CAD), not the field mid-edges.
    private static final double[][] FLOWER_POS = {
        {174.7, 59.4}, {-174.7, -59.4}, {-59.4, 174.7}, {59.4, -174.7}
    };
    private static final double FRAME_COLLIDE_H = 105.0;   // A-frame leg collision height

    // ---- Hive (tipping seesaw, pivots about X axle at world z=109) --------------------------
    private static final double HIVE_PIVOT_Z    = 109.0;
    private static final double HIVE_MASS       = 600.0;
    private static final double HIVE_FLIP       = 0.72;    // rad to swing to the opposite (dumped) stop
    // Pivot "friction" hold torque. Calibrated (headless physics test): holds the 3 staged Nectar,
    // tips at 3 Nectar + 3 Pollen -- matching the real BIOBUZZ hive calibration.
    private static final double HIVE_HOLD_TORQUE = 8.0e6;
    // Cell (basket) centers in body-local coords (world center minus pivot); up cell is elevated.
    private static final double CELL_Y = 30.0, CELL_UP_Z = 22.9, CELL_DN_Z = -11.9, CELL_X = 32.0;

    private static BioBuzzField instance = null;

    private final double HALF;
    private final Image backgroundImage = Config.BACKGROUND;

    private final List<GamePiece> pieces = new ArrayList<>();
    private final List<FxBody> hives = new ArrayList<>();
    private PhongMaterial pollenMat;

    /** A dynamic game piece plus the pose it respawns to on reset. */
    private static class GamePiece {
        final FxBody body;
        final DVector3 spawn;
        GamePiece(FxBody body, DVector3 spawn) { this.body = body; this.spawn = spawn; }
    }

    public static BioBuzzField getInstance(Group group, DWorld world, DSpace space) {
        if (instance == null) {
            instance = new BioBuzzField(group, world, space);
        }
        return instance;
    }

    private BioBuzzField(Group group, DWorld world, DSpace space) {
        super(group, world, space);
        HALF = VirtualRobotController.HALF_FIELD_WIDTH;
    }

    @Override
    public void setup() {
        pollenMat = new PhongMaterial(Color.rgb(240, 190, 40)); // Pollen = yellow
        pollenMat.setSpecularColor(Color.WHITE);

        buildFloorAndWalls();
        buildTapeZones();

        // Real CAD props (already positioned; identity transform).
        addMesh(MESH_DIR + "frame.obj",  Color.rgb(165, 170, 175)); // aluminium
        addMesh(MESH_DIR + "flower.obj", Color.rgb(95, 167, 61));   // green

        // Real hive meshes on hinges -- semi-transparent so the open-basket lattice reads through.
        buildHive(MESH_DIR + "hive_red.obj",  Color.rgb(200, 50, 50, 0.5), true);
        buildHive(MESH_DIR + "hive_blue.obj", Color.rgb(55, 90, 210, 0.5), false);

        buildFrameCollision();
        buildFlowers();
        buildGardenPollen();
        buildAllianceTrays();
    }

    // ---- Tile-grid coordinate helpers (col A=0..F=5, row 1..6) ------------------------------

    private double tileX(int col)  { return -HALF + TILE / 2.0 + col * TILE; }
    private double tileY(int row)  { return -HALF + TILE / 2.0 + (row - 1) * TILE; }

    // ---- Static geometry -------------------------------------------------------------------

    private void addMesh(String resource, Color color) {
        MeshView view = new MeshView(ObjMeshLoader.loadResource(resource));
        PhongMaterial mat = new PhongMaterial(color);
        mat.setSpecularColor(Color.WHITE);
        view.setMaterial(mat);
        view.setCullFace(CullFace.NONE); // decimated meshes can have mixed winding
        subSceneGroup.getChildren().add(view);
    }

    private void buildFloorAndWalls() {
        TriangleMesh fieldMesh = Util3D.getParametricMesh(-HALF, HALF, -HALF, HALF,
                10, 10, false, false, new Util3D.Param3DEqn() {
                    @Override public float x(float s, float t) { return s; }
                    @Override public float y(float s, float t) { return t; }
                    @Override public float z(float s, float t) { return 0; }
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
            TriangleMesh wallMesh = Util3D.getParametricMesh(-HALF, HALF, -15, 15,
                    10, 10, false, false, new Util3D.Param3DEqn() {
                        @Override public float x(float s, float t) { return s; }
                        @Override public float y(float s, float t) { return 0; }
                        @Override public float z(float s, float t) { return t; }
                    });
            MeshView wallView = new MeshView(wallMesh);
            wallView.setMaterial(wallMaterial);
            double dx = i == 0 ? 0 : i == 1 ? -HALF - 2 : i == 2 ? 0 : HALF + 2;
            double dy = i == 0 ? HALF + 2 : i == 1 ? 0 : i == 2 ? -HALF - 2 : 0;
            double angle = i == 0 ? 0 : i == 1 ? 90 : i == 2 ? 180 : -90;
            wallView.getTransforms().addAll(new Translate(dx, dy, 15), new Rotate(angle, Rotate.Z_AXIS));
            subSceneGroup.getChildren().add(wallView);
            List<DGeom> wallGeoms = FxBodyHelper.dGeomsFromNode(wallView, space, null);
            for (DGeom dg : wallGeoms) dg.setCategoryBits(CBits.WALLS);
        }
    }

    /** Painted tape zones -- thin outline lines (like the real gaffer tape), purely visual. */
    private void buildTapeZones() {
        Color red = Color.rgb(210, 40, 40);
        Color blue = Color.rgb(40, 70, 210);
        addFloorOutline(tileX(0), tileY(1), 50, 50, red);            // Red Garden (A1)
        addFloorOutline(tileX(5), tileY(6), 50, 50, blue);           // Blue Garden (F6)
        addFloorOutline(tileX(0), tileY(5), 40, 40, red);            // Red Loading Zone (A5)
        addFloorOutline(tileX(5), tileY(2), 40, 40, blue);           // Blue Loading Zone (F2)
    }

    /** Draw a rectangle outline from four thin tape lines. */
    private void addFloorOutline(double cx, double cy, double w, double h, Color color) {
        double t = 2.5;
        addFloorRect(cx, cy + h / 2, w, t, color);
        addFloorRect(cx, cy - h / 2, w, t, color);
        addFloorRect(cx - w / 2, cy, t, h, color);
        addFloorRect(cx + w / 2, cy, t, h, color);
    }

    private void addFloorRect(double cx, double cy, double w, double h, Color color) {
        Box rect = new Box(w, h, 0.4);
        rect.setMaterial(new PhongMaterial(color));
        rect.getTransforms().add(new Translate(cx, cy, 0.3));
        subSceneGroup.getChildren().add(rect);
    }

    /**
     * Collision for the two A-frame side panels at +/-X. The center (|x| < ~31 cm) is left OPEN so
     * the robot can drive through the middle and under the raised hive.
     */
    private void buildFrameCollision() {
        double[][] legs = {{46.5, 24.75}, {46.5, -24.75}, {-46.5, 24.75}, {-46.5, -24.75}};
        for (double[] leg : legs) {
            DBox box = OdeHelper.createBox(31, 49.5, FRAME_COLLIDE_H);
            space.add(box);
            box.setPosition(leg[0], leg[1], FRAME_COLLIDE_H / 2.0);
            box.setCategoryBits(CBits.STRUCTURE);
            box.setData("Frame Leg");
        }
    }

    // ---- Game-piece placement --------------------------------------------------------------

    /** Flower collision cylinders (so the robot can't drive through) + Pollen staged field-ward of each. */
    private void buildFlowers() {
        for (double[] f : FLOWER_POS) {
            double cx = f[0], cy = f[1];
            DCylinder cyl = OdeHelper.createCylinder(8, 57);
            space.add(cyl);
            cyl.setPosition(cx, cy, 57 / 2.0);   // DCylinder axis is Z -> stands upright
            cyl.setCategoryBits(CBits.STRUCTURE);
            cyl.setData("Flower");

            // Four Pollen just field-ward of the flower (outside the cylinder, reachable by the bot).
            double len = Math.hypot(cx, cy);
            double px = cx - cx / len * 16.0, py = cy - cy / len * 16.0;
            double d = POLLEN_R * 1.1;
            addPollen(px - d, py - d, POLLEN_R + 1);
            addPollen(px + d, py - d, POLLEN_R + 1);
            addPollen(px - d, py + d, POLLEN_R + 1);
            addPollen(px + d, py + d, POLLEN_R + 1);
        }
    }

    private void buildGardenPollen() {
        for (int i = 0; i < 4; i++) {
            addPollen(-HALF + 14 + i * (POLLEN_R * 2.2), -HALF + 14, POLLEN_R + 1); // Red Garden (A1)
            addPollen(HALF - 14 - i * (POLLEN_R * 2.2), HALF - 14, POLLEN_R + 1);   // Blue Garden (F6)
        }
    }

    /** Scoring Element Trays staged in each Alliance Area: 5 Nectar + 4 Pollen per alliance. */
    private void buildAllianceTrays() {
        Color red = Color.rgb(200, 50, 50), blue = Color.rgb(60, 90, 210);
        for (int i = 0; i < 5; i++) {
            addNectar(-HALF + 22, -40 + i * 20, NECTAR_LEN, red);  // Red tray (-x wall)
            addNectar(HALF - 22, -40 + i * 20, NECTAR_LEN, blue);  // Blue tray (+x wall)
        }
        for (int i = 0; i < 4; i++) {
            addPollen(-HALF + 44, -30 + i * 20, POLLEN_R + 1);
            addPollen(HALF - 44, -30 + i * 20, POLLEN_R + 1);
        }
    }

    // ---- Hive (tipping seesaw) -------------------------------------------------------------

    /**
     * One alliance's Hive: the real (pre-tilted) CAD mesh as visual, on an ODE hinge about the X
     * axle at z=109. Box cell trays (collision only) catch game pieces; a pivot "friction" torque
     * holds the start tilt until the raised cell is loaded past the threshold, then it tips to the
     * opposite stop. Red starts with its -Y cell raised, Blue with its +Y cell raised.
     */
    private void buildHive(String meshResource, Color color, boolean red) {
        FxBody hive = FxBody.newInstance(world, space);

        MeshView mv = new MeshView(ObjMeshLoader.loadResource(meshResource));
        PhongMaterial mat = new PhongMaterial(color);
        mat.setSpecularColor(Color.WHITE);
        mv.setMaterial(mat);
        mv.setCullFace(CullFace.NONE);
        hive.setNode(mv, false);                       // mesh is visual only (trimesh collision too heavy)
        subSceneGroup.getChildren().add(mv);

        DMass mass = OdeHelper.createMass();
        mass.setBoxTotal(HIVE_MASS, 60, 120, 60);      // COM at the pivot -> neutral seesaw
        hive.setMass(mass);

        double x = red ? -CELL_X : CELL_X;
        double upY = red ? -CELL_Y : CELL_Y;           // raised cell at start
        double dnY = red ? CELL_Y : -CELL_Y;
        addCellTray(hive, x, upY, CELL_UP_Z);
        addCellTray(hive, x, dnY, CELL_DN_Z);
        hive.setCategoryBits(CBits.STRUCTURE);
        hive.setAngularDamping(0.4);
        hive.setPosition(0, 0, HIVE_PIVOT_Z);

        DHingeJoint joint = OdeHelper.createHingeJoint(world);
        joint.attach(hive, null);                      // hinge to the static world
        joint.setAnchor(0, 0, HIVE_PIVOT_Z);
        joint.setAxis(1, 0, 0);
        if (red) { joint.setParamLoStop(-0.05); joint.setParamHiStop(HIVE_FLIP); }  // flips +X
        else     { joint.setParamLoStop(-HIVE_FLIP); joint.setParamHiStop(0.05); }  // flips -X
        joint.setParamVel(0);
        joint.setParamFMax(HIVE_HOLD_TORQUE);

        hives.add(hive);

        // Stage 3 Nectar in the raised cell (opaque, unlike the see-through hive).
        Color nectarColor = new Color(color.getRed(), color.getGreen(), color.getBlue(), 1.0);
        double nz = HIVE_PIVOT_Z + CELL_UP_Z + 6;
        for (int i = 0; i < 3; i++) addNectar(x + (i - 1) * 7, upY, nz, nectarColor);
    }

    private void addCellTray(FxBody hive, double cx, double cy, double cz) {
        double s = 22, fT = 3, wT = 2, wallH = 8;
        addHiveGeom(hive, s, s, fT, cx, cy, cz);                       // floor
        addHiveGeom(hive, s, wT, wallH, cx, cy + s / 2, cz + wallH / 2);
        addHiveGeom(hive, s, wT, wallH, cx, cy - s / 2, cz + wallH / 2);
        addHiveGeom(hive, wT, s, wallH, cx + s / 2, cy, cz + wallH / 2);
        addHiveGeom(hive, wT, s, wallH, cx - s / 2, cy, cz + wallH / 2);
    }

    private void addHiveGeom(FxBody hive, double lx, double ly, double lz, double x, double y, double z) {
        DBox box = OdeHelper.createBox(lx, ly, lz);
        hive.addGeom(box, x, y, z);
    }

    // ---- Dynamic-piece factories -----------------------------------------------------------

    private void addPollen(double x, double y, double z) {
        FxBody body = FxBody.newInstance(world, space);
        DMass mass = OdeHelper.createMass();
        mass.setSphereTotal(POLLEN_MASS, POLLEN_R);
        body.setMass(mass);

        Sphere sphere = new Sphere(POLLEN_R);
        sphere.setMaterial(pollenMat);
        body.setNode(sphere, true);
        body.setCategoryBits(CBits.GAME_PIECE);
        body.setLinearDamping(0.02);
        body.setAngularDamping(0.05);
        body.setPosition(x, y, z);

        subSceneGroup.getChildren().add(sphere);
        pieces.add(new GamePiece(body, new DVector3(x, y, z)));
    }

    private void addNectar(double x, double y, double z, Color color) {
        FxBody body = FxBody.newInstance(world, space);
        DMass mass = OdeHelper.createMass();
        mass.setSphereTotal(NECTAR_MASS, NECTAR_R); // sphere-approx inertia; stable for a small piece
        body.setMass(mass);

        Cylinder cyl = new Cylinder(NECTAR_R, NECTAR_LEN);
        PhongMaterial mat = new PhongMaterial(color);
        mat.setSpecularColor(Color.WHITE);
        cyl.setMaterial(mat);
        body.setNode(cyl, true);
        body.setCategoryBits(CBits.GAME_PIECE);
        body.setLinearDamping(0.02);
        body.setAngularDamping(0.05);
        body.setPosition(x, y, z);

        subSceneGroup.getChildren().add(cyl);
        pieces.add(new GamePiece(body, new DVector3(x, y, z)));
    }

    // ---- FtcField hooks --------------------------------------------------------------------

    @Override
    public void reset() {
        for (GamePiece p : pieces) {
            p.body.setLinearVel(0, 0, 0);
            p.body.setAngularVel(0, 0, 0);
            p.body.setPosition(p.spawn.get0(), p.spawn.get1(), p.spawn.get2());
        }
        for (FxBody hive : hives) {
            hive.setLinearVel(0, 0, 0);
            hive.setAngularVel(0, 0, 0);
            DMatrix3 identity = new DMatrix3();
            identity.setIdentity();
            hive.setRotation(identity);                // back to the start tilt (mesh is pre-tilted)
            hive.setPosition(0, 0, HIVE_PIVOT_Z);
        }
    }

    @Override
    public void updateDisplay() {
        for (GamePiece p : pieces) p.body.updateNodeDisplay();
        for (FxBody hive : hives) hive.updateNodeDisplay();
    }

    @Override
    public void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup) {
        for (int i = 0; i < numContacts; i++) {
            DContact contact = contacts.get(i);
            contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
            contact.surface.mu = 0.4;
            contact.surface.bounce = 0.3;
            contact.surface.bounce_vel = 1.5;
            contact.surface.soft_cfm = 1e-8;
            contact.surface.soft_erp = 0.3;
            DJoint c = OdeHelper.createContactJoint(world, contactGroup, contact);
            c.attach(contact.geom.g1.getBody(), contact.geom.g2.getBody());
        }
    }
}
