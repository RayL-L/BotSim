package odefx;

public class CBits {

    /**
     * Category Bits
     */

    public static final long FLOOR = 0x1;
    public static final long WALLS = 0x2;

    /**
     * Field game elements. These MUST fall within the 0xF0 mask: VirtualRobotController.nearCallback
     * only routes a contact to FtcField.handleContacts() when one of the colliding geoms has a
     * category bit intersecting 0xF0. (FLOOR/WALLS deliberately do not, so their contacts use the
     * default handler.)
     */
    public static final long GAME_PIECE = 0x10;
    public static final long GOAL = 0x20;
    public static final long STRUCTURE = 0x40;

    public static final long BOT = 0x100;

}
