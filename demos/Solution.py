import sys
import math

EPS = 1e-9

def cross_vec(a, b):
    return a[0] * b[1] - a[1] * b[0]

def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def scale(a, s):
    return (a[0] * s, a[1] * s)

def norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1])

def normalize(a):
    n = norm(a)
    if n < EPS:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)

def perp(a):
    return (-a[1], a[0])

def signed_area(poly):
    n = len(poly)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
    return area / 2.0

def ensure_ccw(poly):
    if signed_area(poly) < 0:
        poly.reverse()
    return poly

def is_convex(poly):
    n = len(poly)
    for i in range(n):
        if cross(poly[i], poly[(i + 1) % n], poly[(i + 2) % n]) < -EPS:
            return False
    return True

def closest_point_on_segment(p, a, b):
    ab = sub(b, a)
    ab_sq = dot(ab, ab)
    if ab_sq < EPS * EPS:
        dx, dy = p[0] - a[0], p[1] - a[1]
        return a, math.sqrt(dx*dx + dy*dy)
    t = dot(sub(p, a), ab) / ab_sq
    t = max(0.0, min(1.0, t))
    cp = (a[0] + ab[0] * t, a[1] + ab[1] * t)
    dx, dy = p[0] - cp[0], p[1] - cp[1]
    return cp, math.sqrt(dx*dx + dy*dy)

def point_in_polygon(pt, poly):
    """1=inside, 0=boundary, -1=outside"""
    n = len(poly)
    wn = 0
    px, py = pt
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        cp = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if abs(cp) < EPS:
            d = (bx - ax) * (px - ax) + (by - ay) * (py - ay)
            l2 = (bx - ax) * (bx - ax) + (by - ay) * (by - ay)
            if -EPS <= d <= l2 + EPS:
                return 0
        if ay <= py:
            if by > py and cp > EPS:
                wn += 1
        else:
            if by <= py and cp < -EPS:
                wn -= 1
    return 1 if wn != 0 else -1

# ============================================================
# Minkowski sum of two convex polygons (both CCW)
# ============================================================

def minkowski_sum_convex(P, Q):
    n, m = len(P), len(Q)
    if n == 0 or m == 0:
        return []

    def bottom_idx(poly):
        idx = 0
        for i in range(1, len(poly)):
            if (poly[i][1] < poly[idx][1] - EPS or
                (abs(poly[i][1] - poly[idx][1]) < EPS and poly[i][0] < poly[idx][0])):
                idx = i
        return idx

    ip = bottom_idx(P)
    iq = bottom_idx(Q)
    P2 = P[ip:] + P[:ip]
    Q2 = Q[iq:] + Q[:iq]

    result = []
    i, j = 0, 0
    while i < n or j < m:
        result.append((P2[i % n][0] + Q2[j % m][0], P2[i % n][1] + Q2[j % m][1]))
        ep = sub(P2[(i + 1) % n], P2[i % n])
        eq = sub(Q2[(j + 1) % m], Q2[j % m])
        c = ep[0] * eq[1] - ep[1] * eq[0]
        if i >= n:
            j += 1
        elif j >= m:
            i += 1
        elif c > EPS:
            i += 1
        elif c < -EPS:
            j += 1
        else:
            i += 1
            j += 1

    cleaned = []
    nr = len(result)
    for i in range(nr):
        a = result[(i - 1) % nr]
        b = result[i]
        c = result[(i + 1) % nr]
        if abs(cross(a, b, c)) > EPS:
            cleaned.append(b)
    return cleaned if cleaned else result

# ============================================================
# Ear-clipping triangulation
# ============================================================

def point_in_triangle_inclusive(p, a, b, c):
    d1 = cross(a, b, p)
    d2 = cross(b, c, p)
    d3 = cross(c, a, p)
    has_neg = (d1 < -EPS) or (d2 < -EPS) or (d3 < -EPS)
    has_pos = (d1 > EPS) or (d2 > EPS) or (d3 > EPS)
    return not (has_neg and has_pos)

def triangulate(poly):
    pts = list(poly)
    n = len(pts)
    if n < 3:
        return []
    if n == 3:
        return [list(pts)]

    indices = list(range(n))
    triangles = []

    for _ in range(n * n):
        ni = len(indices)
        if ni <= 3:
            break
        found = False
        for idx in range(ni):
            pi = (idx - 1) % ni
            ni_idx = (idx + 1) % ni
            a = pts[indices[pi]]
            b = pts[indices[idx]]
            c = pts[indices[ni_idx]]

            if cross(a, b, c) < EPS:
                continue

            ear = True
            for k in range(ni):
                if k == pi or k == idx or k == ni_idx:
                    continue
                if point_in_triangle_inclusive(pts[indices[k]], a, b, c):
                    ear = False
                    break

            if ear:
                triangles.append([a, b, c])
                indices.pop(idx)
                found = True
                break

        if not found:
            break

    if len(indices) == 3:
        triangles.append([pts[indices[0]], pts[indices[1]], pts[indices[2]]])
    return triangles

# ============================================================
# Ray-segment intersection
# ============================================================

def ray_segment_intersect_t(ox, oy, dx, dy, ax, ay, bx, by):
    """
    Find t > 0 where ray (ox+t*dx, oy+t*dy) intersects segment (ax,ay)-(bx,by).
    Returns t or -1 if no intersection.
    """
    ex, ey = bx - ax, by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < EPS:
        return -1.0
    fx, fy = ax - ox, ay - oy
    t = (fx * ey - fy * ex) / denom
    s = (fx * dy - fy * dx) / denom
    if t > EPS and -EPS <= s <= 1.0 + EPS:
        return t
    return -1.0

# ============================================================
# NFP computation
# ============================================================

def compute_nfp_parts(poly_a, poly_b):
    neg_b = [(-p[0], -p[1]) for p in poly_b]
    ensure_ccw(neg_b)

    tris_a = triangulate(list(poly_a))
    tris_neg_b = triangulate(neg_b)

    parts = []
    for ta in tris_a:
        ta_ccw = list(ta)
        ensure_ccw(ta_ccw)
        for tb in tris_neg_b:
            tb_ccw = list(tb)
            ensure_ccw(tb_ccw)
            ms = minkowski_sum_convex(ta_ccw, tb_ccw)
            if ms and len(ms) >= 3:
                parts.append(ms)
    return parts

# ============================================================
# Precompute edge normals for all NFP parts
# ============================================================

def precompute_normals(parts):
    """
    Precompute all unique candidate direction vectors from NFP part edges and vertices.
    Returns a list of normalized direction vectors.
    """
    dir_set = set()
    
    for part in parts:
        n = len(part)
        for i in range(n):
            j = (i + 1) % n
            edge = sub(part[j], part[i])
            normal = normalize(perp(edge))
            if norm(normal) > EPS:
                # Round to reduce duplicates
                key = (round(normal[0], 8), round(normal[1], 8))
                dir_set.add(key)
                dir_set.add((-key[0], -key[1]))
    
    return list(dir_set)

# ============================================================
# Solver
# ============================================================

class Solver:
    def __init__(self, poly_a, poly_b):
        self.poly_a = list(poly_a)
        self.poly_b = list(poly_b)
        ensure_ccw(self.poly_a)
        ensure_ccw(self.poly_b)
        
        self.ref_point = self.poly_b[0]
        self.both_convex = is_convex(self.poly_a) and is_convex(self.poly_b)
        
        if self.both_convex:
            neg_b = [(-p[0], -p[1]) for p in self.poly_b]
            ensure_ccw(neg_b)
            self.nfp = minkowski_sum_convex(list(self.poly_a), neg_b)
            self.nfp_parts = [self.nfp] if self.nfp else []
        else:
            self.nfp_parts = compute_nfp_parts(self.poly_a, self.poly_b)
            self.nfp = None
        
        # Precompute candidate directions for general case
        if not self.both_convex:
            self.base_dirs = precompute_normals(self.nfp_parts)
            # Precompute flattened edges for fast ray intersection
            self.flat_edges = []
            for part_idx, part in enumerate(self.nfp_parts):
                n = len(part)
                for i in range(n):
                    j = (i + 1) % n
                    self.flat_edges.append((part_idx, part[i][0], part[i][1], part[j][0], part[j][1]))
    
    def solve(self, move_vec):
        query = (self.ref_point[0] + move_vec[0], self.ref_point[1] + move_vec[1])
        
        if self.both_convex and self.nfp:
            return self._solve_convex(query)
        elif self.nfp_parts:
            return self._solve_general(query)
        else:
            return (0.0, 0.0)
    
    def _solve_convex(self, query):
        pip = point_in_polygon(query, self.nfp)
        if pip <= 0:
            return (0.0, 0.0)
        
        min_d = float('inf')
        best_cp = None
        n = len(self.nfp)
        for i in range(n):
            j = (i + 1) % n
            cp, d = closest_point_on_segment(query, self.nfp[i], self.nfp[j])
            if d < min_d:
                min_d = d
                best_cp = cp
        
        if best_cp is None:
            return (0.0, 0.0)
        return (best_cp[0] - query[0], best_cp[1] - query[1])
    
    def _solve_general(self, query):
        # Find which parts contain the query
        containing = []
        for i, part in enumerate(self.nfp_parts):
            if point_in_polygon(query, part) > 0:
                containing.append(i)
        
        if not containing:
            return (0.0, 0.0)
        
        containing_set = set(containing)
        qx, qy = query
        
        # Build candidate directions:
        # 1. Precomputed edge normals (from all parts)
        # 2. Directions from query to vertices of containing parts
        dirs = list(self.base_dirs)
        
        for idx in containing:
            part = self.nfp_parts[idx]
            for v in part:
                dx, dy = v[0] - qx, v[1] - qy
                ln = math.sqrt(dx*dx + dy*dy)
                if ln > EPS:
                    key = (round(dx/ln, 8), round(dy/ln, 8))
                    dirs.append(key)
        
        # Deduplicate
        dirs = list(set(dirs))
        
        best_t = float('inf')
        best_dx = 0.0
        best_dy = 0.0
        
        for dx, dy in dirs:
            # For each containing part, find ray exit (first edge intersection with t > 0)
            max_exit_t = 0.0
            valid = True
            
            for idx in containing:
                part = self.nfp_parts[idx]
                n = len(part)
                min_t = float('inf')
                for i in range(n):
                    j = (i + 1) % n
                    t = ray_segment_intersect_t(qx, qy, dx, dy,
                                                 part[i][0], part[i][1],
                                                 part[j][0], part[j][1])
                    if t > EPS and t < min_t:
                        min_t = t
                
                if min_t == float('inf'):
                    valid = False
                    break
                max_exit_t = max(max_exit_t, min_t)
            
            if not valid or max_exit_t < EPS:
                continue
            
            # Check that the exit point is outside ALL parts
            # (not just the containing ones - might enter a new part)
            mx, my = qx + dx * (max_exit_t + EPS * 100), qy + dy * (max_exit_t + EPS * 100)
            
            all_outside = True
            for i, part in enumerate(self.nfp_parts):
                if point_in_polygon((mx, my), part) > 0:
                    all_outside = False
                    break
            
            if all_outside and max_exit_t < best_t:
                best_t = max_exit_t
                best_dx = dx * max_exit_t
                best_dy = dy * max_exit_t
            elif not all_outside:
                # Try to advance further - iteratively exit new parts
                t_current = max_exit_t
                for _ in range(20):
                    mx2, my2 = qx + dx * (t_current + EPS * 100), qy + dy * (t_current + EPS * 100)
                    new_containing = []
                    for i, part in enumerate(self.nfp_parts):
                        if point_in_polygon((mx2, my2), part) > 0:
                            new_containing.append(i)
                    
                    if not new_containing:
                        if t_current < best_t:
                            best_t = t_current
                            best_dx = dx * t_current
                            best_dy = dy * t_current
                        break
                    
                    new_max = t_current
                    failed = False
                    for idx in new_containing:
                        part = self.nfp_parts[idx]
                        n = len(part)
                        min_t = float('inf')
                        for i in range(n):
                            j = (i + 1) % n
                            t = ray_segment_intersect_t(qx, qy, dx, dy,
                                                         part[i][0], part[i][1],
                                                         part[j][0], part[j][1])
                            if t > t_current + EPS and t < min_t:
                                min_t = t
                        
                        if min_t == float('inf'):
                            failed = True
                            break
                        new_max = max(new_max, min_t)
                    
                    if failed or new_max <= t_current + EPS:
                        break
                    t_current = new_max
        
        return (best_dx, best_dy)

# ============================================================
# Main I/O
# ============================================================

def main():
    inp = sys.stdin

    # =============== 1. read polygons ===================
    line = inp.readline()
    try:
        n1, n2 = map(int, line.split())
    except ValueError:
        print('Input data error: wrong polygon vertex count.', file=sys.stderr)
        return

    poly_a = []
    for _ in range(n1):
        x, y = map(float, inp.readline().split())
        poly_a.append((x, y))

    poly_b = []
    for _ in range(n2):
        x, y = map(float, inp.readline().split())
        poly_b.append((x, y))

    # wait for OK to ensure all polygon data is received
    ok = inp.readline().strip()
    if ok != "OK":
        print(f"Error: expected OK, got {ok}", file=sys.stderr)
        return

    # ============== 2. preprocess ===================
    solver = Solver(poly_a, poly_b)

    # send OK after finishing preprocessing
    print("OK")
    sys.stdout.flush()

    # ============== 3. read test points ===================
    m = int(inp.readline())
    test_cases = []
    for _ in range(m):
        x, y = map(float, inp.readline().split())
        test_cases.append((x, y))

    # wait for OK to ensure all test cases are received
    ok = inp.readline().strip()
    if ok != "OK":
        print(f"Error: expected OK, got {ok}", file=sys.stderr)
        return

    # ================ 4. solve and output results ===================
    for vec in test_cases:
        res = solver.solve(vec)
        rx = res[0] + 0.0
        ry = res[1] + 0.0
        sx = f"{rx:.5f}"
        sy = f"{ry:.5f}"
        # Eliminate negative zero from formatted output
        if sx == "-0.00000":
            sx = "0.00000"
        if sy == "-0.00000":
            sy = "0.00000"
        print(f"{sx} {sy}")
        sys.stdout.flush()

    # send OK after outputting all answers
    print("OK")
    sys.stdout.flush()

if __name__ == "__main__":
    main()