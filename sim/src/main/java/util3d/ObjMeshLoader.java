package util3d;

import javafx.scene.paint.Material;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Minimal Wavefront OBJ -> JavaFX {@link TriangleMesh} loader for static field props.
 *
 * Reads only vertex positions ("v") and faces ("f"); ignores normals/texcoords/materials/groups.
 * Faces may use any of the "a", "a/b", "a//c", "a/b/c" index forms (1-based, negative allowed) and
 * n-gons are fan-triangulated. Normals are left to JavaFX (POINT_TEXCOORD auto-generates them for
 * lighting), so a single dummy texcoord is emitted. Intended for the pre-decimated per-part OBJs
 * produced from the BIOBUZZ field CAD -- not a general-purpose importer.
 */
public class ObjMeshLoader {

    public static TriangleMesh load(InputStream in) throws IOException {
        List<float[]> points = new ArrayList<>();
        List<int[]> tris = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.length() < 2) continue;
                char c0 = line.charAt(0);
                if (c0 == 'v' && line.charAt(1) == ' ') {
                    String[] t = line.trim().split("\\s+");
                    points.add(new float[]{
                            Float.parseFloat(t[1]), Float.parseFloat(t[2]), Float.parseFloat(t[3])});
                } else if (c0 == 'f' && line.charAt(1) == ' ') {
                    String[] t = line.trim().split("\\s+");
                    int n = t.length - 1;
                    int[] vi = new int[n];
                    for (int i = 0; i < n; i++) {
                        String tok = t[i + 1];
                        int slash = tok.indexOf('/');
                        int idx = Integer.parseInt(slash >= 0 ? tok.substring(0, slash) : tok);
                        vi[i] = idx > 0 ? idx - 1 : points.size() + idx; // negative = relative to current
                    }
                    for (int i = 1; i + 1 < n; i++) {
                        tris.add(new int[]{vi[0], vi[i], vi[i + 1]});
                    }
                }
            }
        }

        TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_TEXCOORD);
        float[] pts = new float[points.size() * 3];
        for (int i = 0; i < points.size(); i++) {
            float[] p = points.get(i);
            pts[3 * i] = p[0];
            pts[3 * i + 1] = p[1];
            pts[3 * i + 2] = p[2];
        }
        mesh.getPoints().setAll(pts);
        mesh.getTexCoords().setAll(0, 0);

        int[] faces = new int[tris.size() * 6];
        for (int i = 0; i < tris.size(); i++) {
            int[] f = tris.get(i);
            faces[6 * i] = f[0];     faces[6 * i + 1] = 0;
            faces[6 * i + 2] = f[1]; faces[6 * i + 3] = 0;
            faces[6 * i + 4] = f[2]; faces[6 * i + 5] = 0;
        }
        mesh.getFaces().setAll(faces);
        return mesh;
    }

    /** Load an OBJ bundled as a classpath resource (e.g. "/virtual_robot/ftcfield/meshes/frame.obj"). */
    public static TriangleMesh loadResource(String resourcePath) {
        try (InputStream in = ObjMeshLoader.class.getResourceAsStream(resourcePath)) {
            if (in == null) throw new FileNotFoundException("Mesh resource not found: " + resourcePath);
            return load(in);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load mesh: " + resourcePath, e);
        }
    }

    /** Convenience: load a resource OBJ as a MeshView with the given material applied. */
    public static MeshView loadMeshView(String resourcePath, Material material) {
        MeshView view = new MeshView(loadResource(resourcePath));
        view.setMaterial(material);
        return view;
    }
}
