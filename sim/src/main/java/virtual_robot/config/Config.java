package virtual_robot.config;

import javafx.scene.image.Image;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;

/**
 * Class for configuring field (width and image), and gamepad (virtual vs. real)
 */
public class Config {

    /**
     *  Width of the subscene, in pixels
     */
    public static final double SUBSCENE_WIDTH = 800;

    /**
     * Whether to use "Virtual Gamepad" (true -> Virtual gamepad, false -> Real gamepad)
     */
    public static final boolean USE_VIRTUAL_GAMEPAD = true;

    /**
     * The image object for the field: a plain gray 6x6 foam-tile mat matching the real FTC field
     * (medium gray tiles with darker interlocking seams). Alliance/zone markings are drawn as 3D
     * tape outlines in BioBuzzField, not baked into this texture. Also the pixel source for the
     * simulated color sensor (see VirtualRobotController.ColorSensorImpl), so it must be a real
     * readable Image.
     */
    public static final Image BACKGROUND = fieldTileImage(648);

    /** Plain gray tile mat with a 6x6 seam grid. Pure pixel writes (safe at class-load, off FX thread). */
    private static Image fieldTileImage(int size) {
        WritableImage image = new WritableImage(size, size);
        var writer = image.getPixelWriter();

        final Color tile = Color.rgb(122, 124, 127);  // foam gray
        final Color seam = Color.rgb(96, 98, 101);     // darker tile seam
        final double tp = size / 6.0;                  // tile pitch (6x6 grid)
        final double sw = 2.5;                          // seam half-width in px

        for (int py = 0; py < size; py++) {
            for (int px = 0; px < size; px++) {
                double fx = px % tp, fy = py % tp;
                boolean onSeam = fx < sw || fy < sw || fx > tp - sw || fy > tp - sw;
                writer.setColor(px, py, onSeam ? seam : tile);
            }
        }
        return image;
    }

    /*
     * Behavior of virtual gamepad analog controls when they are released.
     *
     * false -> controls "snap back" to zero
     * true -> controls hold their position
     *
     * Either of these behaviors can be overridden by pressing SHIFT or ALT when control is released.
     */
    public static final boolean HOLD_CONTROLS_BY_DEFAULT = true;
}
