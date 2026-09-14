package android.graphics;

/**
 * Provides RGBToHSV and HSVToColor methods with signatures identical to android.graphics.Color
 */
public class Color {

    /**
     * @param red   0..255
     * @param green 0..255
     * @param blue  0..255
     * @param hsv   output: hue 0..360, sat 0..1, value 0..1
     */
    public static void RGBToHSV(int red, int green, int blue, float[] hsv) {
        float r = red / 255f, g = green / 255f, b = blue / 255f;
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        hsv[2] = max;
        hsv[1] = (max == 0f) ? 0f : delta / max;

        if (delta == 0f) {
            hsv[0] = 0f;
        } else if (max == r) {
            hsv[0] = 60f * (((g - b) / delta) % 6f);
        } else if (max == g) {
            hsv[0] = 60f * (((b - r) / delta) + 2f);
        } else {
            hsv[0] = 60f * (((r - g) / delta) + 4f);
        }
        if (hsv[0] < 0f) hsv[0] += 360f;
    }

    public static int HSVToColor(float[] hsv) {
        float h = hsv[0], s = hsv[1], v = hsv[2];
        float c = v * s;
        float x = c * (1f - Math.abs((h / 60f) % 2f - 1f));
        float m = v - c;
        float r, g, b;
        int sector = (int)(h / 60f) % 6;
        switch (sector) {
            case 0: r = c; g = x; b = 0; break;
            case 1: r = x; g = c; b = 0; break;
            case 2: r = 0; g = c; b = x; break;
            case 3: r = 0; g = x; b = c; break;
            case 4: r = x; g = 0; b = c; break;
            default: r = c; g = 0; b = x; break;
        }
        int red   = Math.min(255, (int)((r + m) * 256));
        int green = Math.min(255, (int)((g + m) * 256));
        int blue  = Math.min(255, (int)((b + m) * 256));
        return (red << 16) | (green << 8) | blue;
    }
}
