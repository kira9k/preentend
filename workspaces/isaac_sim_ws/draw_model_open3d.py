"""Build a surface mesh from point clouds aligned in world coordinates.

Example::

    python draw_model_open3d.py /path/to/scan --output model_surface_m.ply --unit-scale 1 --no-gui
    python draw_model_open3d.py /path/to/scan --output model_surface_mm.stl --no-gui
    python draw_model_open3d.py /path/to/scan --method poisson --output model_filled_m.ply --unit-scale 1 --no-gui

The default ball-pivoting method preserves measured surfaces and leaves unseen
areas open. Poisson reconstructs a nearly closed surface and adds a flat
synthetic underside; its filled regions are not camera measurements. The legacy
rings method approximates each slice with a convex hull.
Use separate camera PLY files so their normals can be oriented per viewpoint.
Camera positions 1-5 are defined below. Unknown cameras use approximate axial
viewpoints; use --camera-origin for diagonal views or uncertain placements.
STL, OBJ and PLY contain triangle meshes, not parametric CAD solids.
"""

import argparse
import re
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull, QhullError
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


EXPORT_FORMATS = {".stl", ".obj", ".ply"}

# World coordinates of camera mounts in stages/test_3_cameras.usd, metres.
# These are used only to orient surface normals. If a camera moves, update its
# position here (or pass --camera-origin for this run).
CAMERA_POSITIONS_M = {
    1: (3.0, 0.0, 0.4),
    2: (-3.0, 0.0, 0.4),
    3: (0.0, 0.0, 2.0),
    4: (0.0, 2.0, 0.4),
    5: (0.0, -2.0, 0.4),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", type=Path,
                        help="PLY files in world coordinates or a directory with camera_*_world.ply")
    parser.add_argument("-o", "--output", type=Path, help="Save mesh as STL, OBJ or PLY")
    parser.add_argument("--no-gui", action="store_true", help="Skip the viewer (requires --output)")
    parser.add_argument("--show-cloud", action="store_true", help="Show filtered points with the mesh")
    parser.add_argument("--method", choices=("bpa", "poisson", "rings"), default="bpa",
                        help="Surface reconstruction method (default: bpa)")

    geometry = parser.add_argument_group("reconstruction (input units: metres)")
    geometry.add_argument("--min-bound", nargs=3, type=float, metavar=("X", "Y", "Z"),
                          default=(-2, -2, 0.0))
    geometry.add_argument("--max-bound", nargs=3, type=float, metavar=("X", "Y", "Z"),
                          default=(2, 2, 1.0))
    geometry.add_argument("--floor-margin", type=float, default=0.01,
                          help="Discard points within this distance above min Z (default: 0.01 m)")
    geometry.add_argument("--voxel-size", type=float,
                          help="Downsampling voxel edge (default: 0.006 m for bpa/poisson, 0.002 m for rings)")
    geometry.add_argument("--normal-radius", type=float,
                          help="Normal estimation radius for bpa (default: 4 times voxel size)")
    geometry.add_argument("--ball-radii", nargs="+", type=float, metavar="R",
                          help="Ball pivoting radii (default: 1.5, 3 and 6 times voxel size)")
    geometry.add_argument("--min-component-triangles", type=int, default=100,
                          help="Discard disconnected patches smaller than this (default: 100)")
    geometry.add_argument("--xy-padding", type=float, default=0.05,
                          help="Padding around robust XY object bounds (default: 0.05 m)")
    geometry.add_argument("--no-auto-xy-trim", action="store_true",
                          help="Keep points outside automatically estimated XY object bounds")
    geometry.add_argument("--camera-origin", nargs=3, type=float, action="append", metavar=("X", "Y", "Z"),
                          help="Override configured/inferred viewpoint; repeat once per input cloud in filename order")
    geometry.add_argument("--poisson-depth", type=int, default=9,
                          help="Poisson octree depth (default: 9; higher uses more memory)")
    geometry.add_argument("--poisson-scale", type=float, default=1.1,
                          help="Poisson bounding cube scale (default: 1.1)")
    geometry.add_argument("--base-z", type=float, default=0.14,
                          help="Poisson only: approximate underbody height above the ground "
                               "(default: 0.14 m for this car; preserves clearance between wheels)")
    geometry.add_argument("--base-band", type=float, default=0.22,
                          help="Poisson only: use points below base Z plus this height for footprint (default: 0.22 m)")
    geometry.add_argument("--base-spacing", type=float, default=0.012,
                          help="Poisson only: synthetic underside point spacing (default: 0.012 m)")
    geometry.add_argument("--no-base", action="store_true",
                          help="Poisson only: omit the synthetic underside")
    geometry.add_argument("--layer-height", type=float, default=0.005,
                          help="Horizontal slice height (default: 0.005 m)")
    geometry.add_argument("--ring-points", type=int, default=128,
                          help="Vertices on each contour (default: 128)")
    geometry.add_argument("--close-top", action="store_true", help="Rings only: cap the upper opening")
    geometry.add_argument("--open-bottom", action="store_true", help="Rings only: leave the bottom open")
    parser.add_argument("--unit-scale", type=float, default=1000.0,
                        help="Export coordinate multiplier; 1000 converts metres to millimetres (default: 1000)")

    args = parser.parse_args()
    if args.no_gui and args.output is None:
        parser.error("--no-gui requires --output")
    if args.output and args.output.suffix.lower() not in EXPORT_FORMATS:
        parser.error("--output must end in .stl, .obj or .ply")
    if any(lo >= hi for lo, hi in zip(args.min_bound, args.max_bound)):
        parser.error("each --min-bound coordinate must be less than --max-bound")
    if args.voxel_size is None:
        args.voxel_size = 0.002 if args.method == "rings" else 0.006
    if args.floor_margin < 0 or args.voxel_size <= 0 or args.layer_height <= 0:
        parser.error("floor margin must be nonnegative; voxel size and layer height must be positive")
    if args.ring_points < 3 or args.unit_scale <= 0:
        parser.error("ring points must be at least 3; unit scale must be positive")
    if args.normal_radius is not None and args.normal_radius <= 0:
        parser.error("--normal-radius must be positive")
    if args.ball_radii is not None and any(radius <= 0 for radius in args.ball_radii):
        parser.error("--ball-radii must be positive")
    if args.min_component_triangles < 0:
        parser.error("--min-component-triangles cannot be negative")
    if args.xy_padding < 0:
        parser.error("--xy-padding cannot be negative")
    if not 5 <= args.poisson_depth <= 12 or args.poisson_scale <= 1:
        parser.error("Poisson depth must be 5..12 and scale must exceed 1")
    if args.base_band <= 0 or args.base_spacing <= 0:
        parser.error("base band and spacing must be positive")
    return args


def load_clouds(inputs: list[Path]) -> list[tuple[Path, o3d.geometry.PointCloud]]:
    paths = []
    for source in inputs:
        if source.is_dir():
            matches = sorted(source.glob("camera_*_world.ply"))
            if not matches:
                raise ValueError(f"No camera_*_world.ply files in {source}")
            paths.extend(matches)
        elif source.is_file():
            paths.append(source)
        else:
            raise FileNotFoundError(source)

    clouds = []
    for path in paths:
        cloud = o3d.io.read_point_cloud(str(path))
        if cloud.is_empty():
            raise ValueError(f"Empty or unreadable cloud: {path}")
        print(f"Loaded {path}: {len(cloud.points)} points")
        clouds.append((path, cloud))
    return clouds


def combine_clouds(clouds: list[tuple[Path, o3d.geometry.PointCloud]]) -> o3d.geometry.PointCloud:
    combined = o3d.geometry.PointCloud()
    for _, cloud in clouds:
        combined += cloud
    return combined


def infer_camera_origin(
    cloud: o3d.geometry.PointCloud, object_center: np.ndarray, half_extent: np.ndarray
) -> np.ndarray:
    """Choose a viewpoint outside the object on the observed side.

    The exact camera position is unnecessary for flipping normals; only its
    direction from the observed surface matters. This approximation assumes
    the input cloud was cropped to one object and cameras are roughly axial.
    """
    points = np.asarray(cloud.points)
    observed_center = np.median(points, axis=0)
    # Normalize by object dimensions: otherwise the long car axis can drown
    # out a smaller but decisive height difference for the overhead camera.
    direction = (observed_center - object_center) / half_extent
    axis = int(np.argmax(np.abs(direction)))
    if abs(direction[axis]) < 0.05:
        raise ValueError("Cannot infer a camera viewpoint; pass --camera-origin for each cloud")
    viewpoint = object_center.copy()
    viewpoint[axis] += np.sign(direction[axis]) * 3.0
    return viewpoint


def camera_origin_for(
    path: Path, cloud: o3d.geometry.PointCloud, object_center: np.ndarray,
    half_extent: np.ndarray, allow_inference: bool
) -> tuple[np.ndarray, str]:
    match = re.fullmatch(r"camera_(\d+)_world\.ply", path.name)
    camera_number = int(match.group(1)) if match else None
    if camera_number in CAMERA_POSITIONS_M:
        return np.asarray(CAMERA_POSITIONS_M[camera_number], dtype=np.float64), "configured"
    if not allow_inference:
        raise ValueError(f"No camera position for {path}; pass --camera-origin X Y Z")
    return infer_camera_origin(cloud, object_center, half_extent), "inferred"


def prepare_oriented_cloud(
    clouds: list[tuple[Path, o3d.geometry.PointCloud]], args: argparse.Namespace
) -> o3d.geometry.PointCloud:
    """Crop, filter and orient each camera cloud before merging them."""
    if args.camera_origin is not None and len(args.camera_origin) != len(clouds):
        raise ValueError("Repeat --camera-origin exactly once for each input cloud")

    bounds = o3d.geometry.AxisAlignedBoundingBox(args.min_bound, args.max_bound)
    normal_radius = args.normal_radius or 4 * args.voxel_size
    prepared = []

    for index, (path, raw) in enumerate(clouds):
        # Drop invalid and floor points before crop/downsampling. Keep each camera
        # separate until its normals have been oriented toward its own viewpoint.
        points = np.asarray(raw.points)
        valid = np.isfinite(points).all(axis=1) & (points[:, 2] > args.min_bound[2] + args.floor_margin)
        cloud = raw.select_by_index(np.flatnonzero(valid).tolist()).crop(bounds)
        cloud = cloud.voxel_down_sample(args.voxel_size)
        if len(cloud.points) < 50:
            raise ValueError(f"Too few points from {path} after cropping and downsampling")
        cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
        if len(cloud.points) < 50:
            raise ValueError(f"Too few points from {path} after filtering")
        print(f"Filtered {path.name}: {len(cloud.points)} points")
        prepared.append((path, cloud))

    # Side and end cameras can retain coherent pieces of the surrounding scene.
    # Find the main object's XY extent from the pooled cloud and trim those
    # pieces before triangulation. The padding preserves sparse edge details.
    all_points = np.concatenate([np.asarray(cloud.points) for _, cloud in prepared])
    lower = np.quantile(all_points, 0.01, axis=0)
    upper = np.quantile(all_points, 0.99, axis=0)
    if not args.no_auto_xy_trim:
        xy_lower = lower[:2] - args.xy_padding
        xy_upper = upper[:2] + args.xy_padding
        trimmed = []
        for path, cloud in prepared:
            points = np.asarray(cloud.points)
            inside = ((points[:, :2] >= xy_lower) & (points[:, :2] <= xy_upper)).all(axis=1)
            cloud = cloud.select_by_index(np.flatnonzero(inside).tolist())
            if len(cloud.points) < 50:
                raise ValueError(f"Too few points from {path} after XY trim; increase --xy-padding")
            trimmed.append((path, cloud))
        prepared = trimmed
        print(f"Auto XY bounds: {xy_lower.round(3).tolist()} to {xy_upper.round(3).tolist()}")
        all_points = np.concatenate([np.asarray(cloud.points) for _, cloud in prepared])
        lower = np.quantile(all_points, 0.01, axis=0)
        upper = np.quantile(all_points, 0.99, axis=0)
    object_center = (lower + upper) / 2
    half_extent = np.maximum((upper - lower) / 2, 0.01)
    filtered = o3d.geometry.PointCloud()
    for index, (path, cloud) in enumerate(prepared):
        cloud.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30)
        )
        if args.camera_origin is not None:
            origin = np.asarray(args.camera_origin[index], dtype=np.float64)
            source = "command line"
        else:
            origin, source = camera_origin_for(
                path, cloud, object_center, half_extent, allow_inference=len(prepared) > 1
            )
        cloud.orient_normals_towards_camera_location(origin)
        print(f"Viewpoint for {path.name} ({source}): {np.round(origin, 3).tolist()}")
        filtered += cloud
    return filtered


def build_bpa(cloud: o3d.geometry.PointCloud, args: argparse.Namespace) -> o3d.geometry.TriangleMesh:
    """Reconstruct only surfaces supported by the camera measurements."""
    radii = args.ball_radii or [factor * args.voxel_size for factor in (1.5, 3, 6)]

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, o3d.utility.DoubleVector(sorted(radii))
    )
    if len(mesh.triangles) == 0:
        raise ValueError("No triangles reconstructed; adjust voxel size or ball radii")

    if args.min_component_triangles:
        labels, counts, _ = mesh.cluster_connected_triangles()
        labels = np.asarray(labels)
        counts = np.asarray(counts)
        small = counts[labels] < args.min_component_triangles
        if small.any():
            mesh.remove_triangles_by_mask(small.tolist())
            mesh.remove_unreferenced_vertices()
        print(f"Removed {int(small.sum())} triangles from small disconnected patches")

    mesh.remove_degenerate_triangles()
    mesh.compute_vertex_normals()
    boundary_edges = len(mesh.get_non_manifold_edges(allow_boundary_edges=False))
    non_manifold_vertices = len(mesh.get_non_manifold_vertices())
    print(f"Reconstructed {len(mesh.triangles)} triangles from {len(cloud.points)} filtered points")
    print(f"Open boundary edges: {boundary_edges}; non-manifold vertices: {non_manifold_vertices}")
    return mesh


def add_approximate_base(cloud: o3d.geometry.PointCloud, args: argparse.Namespace) -> o3d.geometry.PointCloud:
    """Add a synthetic horizontal underside inside the low-point XY footprint."""
    points = np.asarray(cloud.points)
    low_points = points[points[:, 2] < args.base_z + args.base_band]
    if len(low_points) < 50:
        raise ValueError("Too few low points for a base; increase --base-band")
    try:
        footprint = ConvexHull(low_points[:, :2])
    except QhullError as exc:
        raise ValueError("Cannot infer the base footprint from low points") from exc

    x = np.arange(low_points[:, 0].min(), low_points[:, 0].max(), args.base_spacing)
    y = np.arange(low_points[:, 1].min(), low_points[:, 1].max(), args.base_spacing)
    grid_x, grid_y = np.meshgrid(x, y)
    xy = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    # Keep the synthetic points inside the hull, away from its outermost edge.
    inside = np.all(
        xy @ footprint.equations[:, :2].T + footprint.equations[:, 2]
        <= -0.3 * args.base_spacing, axis=1
    )
    xy = xy[inside]
    if len(xy) < 50:
        raise ValueError("Base footprint is too small; reduce --base-spacing")

    base = o3d.geometry.PointCloud()
    base.points = o3d.utility.Vector3dVector(
        np.column_stack((xy, np.full(len(xy), args.base_z)))
    )
    base.normals = o3d.utility.Vector3dVector(
        np.tile((0.0, 0.0, -1.0), (len(xy), 1))
    )
    print(f"Added {len(xy)} synthetic underbody points at Z={args.base_z:g} m "
          "(this surface is approximated, not measured)")
    return cloud + base


def build_poisson(cloud: o3d.geometry.PointCloud, args: argparse.Namespace) -> o3d.geometry.TriangleMesh:
    """Smoothly approximate unobserved areas, optionally including the underside."""
    if not args.no_base:
        cloud = add_approximate_base(cloud, args)
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud, depth=args.poisson_depth, scale=args.poisson_scale
    )
    if len(mesh.triangles) == 0:
        raise ValueError("Poisson reconstruction produced no triangles")
    mesh.compute_vertex_normals()
    print(f"Poisson mesh: {len(mesh.triangles)} triangles; "
          f"open boundary edges: {len(mesh.get_non_manifold_edges(allow_boundary_edges=False))}")
    return mesh


def contour_from_layer(layer: np.ndarray, directions: np.ndarray) -> np.ndarray | None:
    """Sample a layer's convex hull at the same angles used by other rings."""
    if len(layer) < 6:
        return None
    xy = np.unique(layer[:, :2], axis=0)
    if len(xy) < 3:
        return None
    try:
        hull = ConvexHull(xy)
    except QhullError:
        return None
    if hull.volume < 1e-8:
        return None

    center = xy[hull.vertices].mean(axis=0)
    normals = hull.equations[:, :2]
    numerator = -(normals @ center + hull.equations[:, 2])
    denominator = normals @ directions.T
    distances = np.full(denominator.shape, np.inf)
    np.divide(numerator[:, None], denominator, out=distances, where=denominator > 1e-12)
    radius = distances.min(axis=0)
    if not np.isfinite(radius).all() or np.any(radius <= 0):
        return None
    ring_xy = center + directions * radius[:, None]
    return np.column_stack((ring_xy, np.full(len(directions), np.median(layer[:, 2]))))


def mesh_from_rings(rings: list[np.ndarray], close_bottom: bool, close_top: bool) -> o3d.geometry.TriangleMesh:
    count = len(rings[0])
    vertices = np.concatenate(rings).tolist()
    triangles = []
    for level in range(len(rings) - 1):
        lower, upper = level * count, (level + 1) * count
        for j in range(count):
            next_j = (j + 1) % count
            triangles.extend(((lower + j, lower + next_j, upper + j),
                              (lower + next_j, upper + next_j, upper + j)))

    if close_bottom:
        center = len(vertices)
        vertices.append(rings[0].mean(axis=0).tolist())
        triangles.extend((center, (j + 1) % count, j) for j in range(count))
    if close_top:
        center = len(vertices)
        vertices.append(rings[-1].mean(axis=0).tolist())
        start = (len(rings) - 1) * count
        triangles.extend((center, start + j, start + (j + 1) % count) for j in range(count))

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(vertices, dtype=np.float64)),
        o3d.utility.Vector3iVector(np.asarray(triangles, dtype=np.int32)),
    )
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh


def prepare_object(cloud: o3d.geometry.PointCloud, args: argparse.Namespace):
    bounds = o3d.geometry.AxisAlignedBoundingBox(args.min_bound, args.max_bound)
    cloud = cloud.crop(bounds)
    points = np.asarray(cloud.points)
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > args.min_bound[2] + args.floor_margin)
    cloud = cloud.select_by_index(np.flatnonzero(valid).tolist())
    if len(cloud.points) < 50:
        raise ValueError("Too few points after cropping; adjust bounds or floor margin")

    cloud = cloud.voxel_down_sample(args.voxel_size)
    cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
    points = np.asarray(cloud.points)
    if len(points) < 50:
        raise ValueError("Too few points after filtering; reduce voxel size or use denser input")

    angles = np.linspace(0, 2 * np.pi, args.ring_points, endpoint=False)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    layer_ids = np.floor((points[:, 2] - points[:, 2].min()) / args.layer_height).astype(int)
    rings = []
    for layer_id in np.unique(layer_ids):
        ring = contour_from_layer(points[layer_ids == layer_id], directions)
        if ring is not None:
            rings.append(ring)
    if len(rings) < 2:
        raise ValueError("Could not build two contours; try a larger --layer-height")

    mesh = mesh_from_rings(rings, close_bottom=not args.open_bottom, close_top=args.close_top)
    print(f"Filtered points: {len(points)}; contours: {len(rings)}; triangles: {len(mesh.triangles)}")
    return cloud, mesh


def save_mesh(mesh: o3d.geometry.TriangleMesh, path: Path, scale: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    export_mesh = o3d.geometry.TriangleMesh(mesh)
    export_mesh.scale(scale, center=(0, 0, 0))
    export_mesh.compute_triangle_normals()
    if not o3d.io.write_triangle_mesh(str(path), export_mesh, write_ascii=False):
        raise OSError(f"Could not write {path}")
    print(f"Saved {path} (coordinates multiplied by {scale:g})")


def show_mesh(mesh: o3d.geometry.TriangleMesh, cloud: o3d.geometry.PointCloud | None = None) -> None:
    app = gui.Application.instance
    app.initialize()
    window = app.create_window("Reconstructed model", 1280, 800)
    scene = gui.SceneWidget()
    scene.scene = rendering.Open3DScene(window.renderer)
    scene.scene.set_background([0.1, 0.1, 0.15, 1.0])
    window.add_child(scene)
    window.set_on_layout(lambda _: setattr(scene, "frame", window.content_rect))

    display_mesh = o3d.geometry.TriangleMesh(mesh)
    display_mesh.paint_uniform_color([0.8, 0.75, 0.65])
    material = rendering.MaterialRecord()
    material.shader = "defaultLit"
    scene.scene.add_geometry("model", display_mesh, material)
    if cloud is not None:
        points = o3d.geometry.PointCloud(cloud)
        points.paint_uniform_color([0.9, 0.4, 0.3])
        point_material = rendering.MaterialRecord()
        point_material.shader = "defaultUnlit"
        point_material.point_size = 2.0
        scene.scene.add_geometry("points", points, point_material)

    bounds = display_mesh.get_axis_aligned_bounding_box()
    scene.setup_camera(60.0, bounds, bounds.get_center())
    app.run()


def main() -> None:
    args = parse_args()
    clouds = load_clouds(args.inputs)
    if args.method == "rings":
        filtered_cloud, mesh = prepare_object(combine_clouds(clouds), args)
    else:
        filtered_cloud = prepare_oriented_cloud(clouds, args)
        mesh = build_bpa(filtered_cloud, args) if args.method == "bpa" else build_poisson(filtered_cloud, args)
    if args.output:
        save_mesh(mesh, args.output, args.unit_scale)
    if not args.no_gui:
        show_mesh(mesh, filtered_cloud if args.show_cloud else None)


if __name__ == "__main__":
    main()
