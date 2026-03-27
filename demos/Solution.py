import sys
from typing import List, Tuple

try:
    from shapely.affinity import translate
    from shapely.geometry import MultiPoint, Point, Polygon
    from shapely.ops import nearest_points, triangulate, unary_union
except Exception as exc:  # pragma: no cover
    print(f"Import error: {exc}", file=sys.stderr)
    raise


EPS = 1e-9
OUTPUT_EPS = 5e-7


class Vector2D:
    __slots__ = ("x", "y")

    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y

    def __add__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.x - other.x, self.y - other.y)


# 全局数据
n1 = 0
n2 = 0
m = 0
polygon1_vertices: List[Vector2D] = []
polygon2_vertices: List[Vector2D] = []
test_cases: List[Vector2D] = []

# 预处理结果
poly_a = None
poly_b = None
nfp_region = None
nfp_boundary = None


def _signed_area(vertices: List[Vector2D]) -> float:
    area = 0.0
    n = len(vertices)
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y
    return area * 0.5



def _ensure_ccw(vertices: List[Vector2D]) -> List[Vector2D]:
    if _signed_area(vertices) < 0.0:
        return list(reversed(vertices))
    return vertices[:]



def _to_shapely_polygon(vertices: List[Vector2D]) -> Polygon:
    coords = [(v.x, v.y) for v in _ensure_ccw(vertices)]
    return Polygon(coords).buffer(0)



def _triangle_coords(tri: Polygon) -> List[Tuple[float, float]]:
    coords = list(tri.exterior.coords)
    if len(coords) >= 4:
        coords = coords[:-1]
    return coords



def _negate_polygon(poly: Polygon) -> Polygon:
    coords = [(-x, -y) for x, y in list(poly.exterior.coords)[:-1]]
    return Polygon(coords).buffer(0)



def _triangulate_inside(poly: Polygon):
    # shapely.ops.triangulate 返回 Delaunay 三角形，需要过滤掉多边形外部部分
    pieces = []
    for tri in triangulate(poly):
        inter = tri.intersection(poly)
        if inter.is_empty:
            continue
        if inter.geom_type == "Polygon" and inter.area > EPS:
            pieces.append(inter)
        elif inter.geom_type == "MultiPolygon":
            for g in inter.geoms:
                if g.area > EPS:
                    pieces.append(g)
    return pieces



def _convex_minkowski_sum(poly1: Polygon, poly2: Polygon) -> Polygon:
    pts1 = _triangle_coords(poly1)
    pts2 = _triangle_coords(poly2)
    summed = [(x1 + x2, y1 + y2) for x1, y1 in pts1 for x2, y2 in pts2]
    hull = MultiPoint(summed).convex_hull
    if hull.geom_type == "Polygon":
        return hull
    # 退化情形理论上不会出现，因为输入简单多边形且任意三个顶点不共线
    return hull.buffer(0)



def _build_nfp(poly_a_obj: Polygon, poly_b_obj: Polygon):
    tris_a = _triangulate_inside(poly_a_obj)
    tris_b = _triangulate_inside(poly_b_obj)

    neg_tris_b = [_negate_polygon(tri) for tri in tris_b]
    pieces = []
    for ta in tris_a:
        for tb in neg_tris_b:
            piece = _convex_minkowski_sum(ta, tb)
            if not piece.is_empty and piece.area > EPS:
                pieces.append(piece)

    if not pieces:
        return Polygon()

    region = unary_union(pieces).buffer(0)
    return region



def pre_process():
    global poly_a, poly_b, nfp_region, nfp_boundary

    poly_a = _to_shapely_polygon(polygon1_vertices)
    poly_b = _to_shapely_polygon(polygon2_vertices)

    nfp_region = _build_nfp(poly_a, poly_b)
    nfp_boundary = nfp_region.boundary



def gen_solution(vec: Vector2D) -> Vector2D:
    if nfp_region.is_empty:
        return Vector2D(0.0, 0.0)

    p = Point(vec.x, vec.y)

    # 只有当平移向量位于 NFP 内部时，两多边形内部才重叠
    if not nfp_region.contains(p):
        return Vector2D(0.0, 0.0)

    nearest = nearest_points(p, nfp_boundary)[1]
    dx = nearest.x - vec.x
    dy = nearest.y - vec.y

    if abs(dx) < OUTPUT_EPS:
        dx = 0.0
    if abs(dy) < OUTPUT_EPS:
        dy = 0.0

    return Vector2D(dx, dy)



def _parse_vertices_from_tokens(tokens: List[str], cnt: int) -> List[Vector2D]:
    if len(tokens) != cnt * 2:
        raise ValueError(f"expect {cnt * 2} float numbers but get {len(tokens)}")
    vals = list(map(float, tokens))
    return [Vector2D(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)]



def _read_polygons_flexibly(cnt1: int, cnt2: int):
    """
    兼容两种输入：
    1) demo 版本：每个顶点各占一行；
    2) 任务书版本：A、B 各自整行给出所有坐标。
    这样不会破坏 demo 的输入输出格式，同时也能直接跑官方样例。
    """
    first = sys.stdin.readline()
    if not first:
        raise EOFError
    tokens = first.split()

    if len(tokens) == cnt1 * 2:
        poly1 = _parse_vertices_from_tokens(tokens, cnt1)
        second = sys.stdin.readline()
        if not second:
            raise EOFError
        poly2 = _parse_vertices_from_tokens(second.split(), cnt2)
        return poly1, poly2

    if len(tokens) != 2:
        raise ValueError("unsupported polygon input format")

    poly1 = [Vector2D(float(tokens[0]), float(tokens[1]))]
    for _ in range(cnt1 - 1):
        line = sys.stdin.readline()
        if not line:
            raise EOFError
        x, y = map(float, line.split())
        poly1.append(Vector2D(x, y))

    poly2 = []
    for _ in range(cnt2):
        line = sys.stdin.readline()
        if not line:
            raise EOFError
        x, y = map(float, line.split())
        poly2.append(Vector2D(x, y))

    return poly1, poly2



def main():
    global n1, n2, m, polygon1_vertices, polygon2_vertices, test_cases

    line = sys.stdin.readline()
    if not line:
        return

    try:
        n1, n2 = map(int, line.split())
    except ValueError:
        print("Input data error: wrong polygon vertex count.", file=sys.stderr)
        return

    try:
        polygon1_vertices, polygon2_vertices = _read_polygons_flexibly(n1, n2)
    except Exception as exc:
        print(f"Input data error while reading polygons: {exc}", file=sys.stderr)
        return

    ok_response = sys.stdin.readline().strip()
    if ok_response != "OK":
        print(
            f"Input data error: waiting for OK after obtaining polygons but I get {ok_response}",
            file=sys.stderr,
        )
        return

    pre_process()
    print("OK")
    sys.stdout.flush()

    line = sys.stdin.readline()
    if not line:
        print("Input data error: missing test case count.", file=sys.stderr)
        return

    m = int(line)
    for _ in range(m):
        x, y = map(float, sys.stdin.readline().split())
        test_cases.append(Vector2D(x, y))

    ok_response = sys.stdin.readline().strip()
    if ok_response != "OK":
        print(
            "Input data error: waiting for OK after that I have received all test points "
            f"but I get {ok_response}",
            file=sys.stderr,
        )
        return


    for test_case in test_cases:
        res = gen_solution(test_case)
        print(f"{res.x:.5f} {res.y:.5f}")
        sys.stdout.flush()

    print("OK")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
