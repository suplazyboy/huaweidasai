#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cfloat>
using namespace std;

const double EPS = 1e-9;

struct V {
    double x, y;
    V(double x = 0, double y = 0) :x(x), y(y) {}
    V operator+(V b) const { return { x + b.x,y + b.y }; }
    V operator-(V b) const { return { x - b.x,y - b.y }; }
    V operator*(double s) const { return { x * s,y * s }; }
    double dot(V b) const { return x * b.x + y * b.y; }
    double cross(V b) const { return x * b.y - y * b.x; }
    double norm2() const { return x * x + y * y; }
    double norm() const { return sqrt(norm2()); }
    V perp() const { return { -y,x }; }
    V unit() const { double n = norm(); return n < EPS ? V(0, 0) : V(x / n, y / n); }
};
typedef vector<V> Poly;
inline double cross3(V o, V a, V b) { return(a - o).cross(b - o); }

// ---- Polygon utilities ----
double signedArea(const Poly& p) {
    double a = 0; int n = p.size();
    for (int i = 0; i < n; i++) { int j = (i + 1) % n; a += p[i].x * p[j].y - p[j].x * p[i].y; }
    return a / 2;
}
void ensureCCW(Poly& p) { if (signedArea(p) < 0)reverse(p.begin(), p.end()); }
bool isConvex(const Poly& p) {
    int n = p.size();
    for (int i = 0; i < n; i++) if (cross3(p[i], p[(i + 1) % n], p[(i + 2) % n]) < -EPS)return false;
    return true;
}

// point-in-polygon: 1=inside, 0=boundary, -1=outside
int pip(double px, double py, const V* poly, int n) {
    int wn = 0;
    for (int i = 0; i < n; i++) {
        double ax = poly[i].x, ay = poly[i].y;
        double bx = poly[(i + 1) % n].x, by = poly[(i + 1) % n].y;
        double cp = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
        if (fabs(cp) < EPS) {
            double d = (bx - ax) * (px - ax) + (by - ay) * (py - ay);
            double l2 = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
            if (d >= -EPS && d <= l2 + EPS)return 0;
        }
        if (ay <= py) { if (by > py && cp > EPS)wn++; }
        else { if (by <= py && cp < -EPS)wn--; }
    }
    return wn ? 1 : -1;
}
inline int pip(V pt, const Poly& p) { return pip(pt.x, pt.y, p.data(), p.size()); }

// closest point on segment, returns dist^2
double closestOnSeg(V p, V a, V b, V& cp) {
    V ab = b - a; double ab2 = ab.dot(ab);
    if (ab2 < EPS * EPS) { cp = a; return(p - a).norm2(); }
    double t = (p - a).dot(ab) / ab2;
    if (t < 0)t = 0; if (t > 1)t = 1;
    cp = a + ab * t; return(p - cp).norm2();
}

// ---- Minkowski sum of two convex CCW polygons ----
Poly minkowskiSum(const Poly& P, const Poly& Q) {
    int n = P.size(), m = Q.size();
    if (!n || !m)return{};
    auto bot = [](const Poly& p)->int {
        int idx = 0;
        for (int i = 1; i < (int)p.size(); i++)
            if (p[i].y < p[idx].y - EPS || (fabs(p[i].y - p[idx].y) < EPS && p[i].x < p[idx].x))idx = i;
        return idx;
        };
    int ip = bot(P), iq = bot(Q);
    Poly P2(n), Q2(m);
    for (int i = 0; i < n; i++)P2[i] = P[(ip + i) % n];
    for (int i = 0; i < m; i++)Q2[i] = Q[(iq + i) % m];
    Poly res; res.reserve(n + m);
    int i = 0, j = 0;
    while (i < n || j < m) {
        res.push_back(P2[i % n] + Q2[j % m]);
        V ep = P2[(i + 1) % n] - P2[i % n], eq = Q2[(j + 1) % m] - Q2[j % m];
        double c = ep.cross(eq);
        if (i >= n)j++; else if (j >= m)i++;
        else if (c > EPS)i++; else if (c < -EPS)j++; else { i++; j++; }
    }
    Poly cl; int nr = res.size();
    for (int i = 0; i < nr; i++) {
        if (fabs(cross3(res[(i - 1 + nr) % nr], res[i], res[(i + 1) % nr])) > EPS)
            cl.push_back(res[i]);
    }
    return cl.empty() ? res : cl;
}

// ---- Ear-clipping triangulation (Optimized) ----
bool ptInTriInc(V p, V a, V b, V c) {
    double d1 = cross3(a, b, p), d2 = cross3(b, c, p), d3 = cross3(c, a, p);
    return!((d1 < -EPS || d2 < -EPS || d3 < -EPS) && (d1 > EPS || d2 > EPS || d3 > EPS));
}
vector<Poly> triangulate(const Poly& poly) {
    int n = poly.size();
    if (n < 3)return{}; if (n == 3)return{ poly };
    vector<int> idx(n); for (int i = 0; i < n; i++)idx[i] = i;
    vector<Poly> tris; tris.reserve(n - 2);
    
    for (int iter = 0; iter < n * n && (int)idx.size() > 3; iter++) {
        int ni = idx.size(); bool found = false;
        for (int ii = 0; ii < ni; ii++) {
            int pi = (ii - 1 + ni) % ni, ni2 = (ii + 1) % ni;
            V a = poly[idx[pi]], b = poly[idx[ii]], c = poly[idx[ni2]];
            if (cross3(a, b, c) < EPS)continue;
            bool ear = true;
            for (int k = 0; k < ni; k++) {
                if (k == pi || k == ii || k == ni2)continue;
                
                // Optimization: Only check reflex vertices in the current polygon
                int k_prev = (k - 1 + ni) % ni;
                int k_next = (k + 1) % ni;
                if (cross3(poly[idx[k_prev]], poly[idx[k]], poly[idx[k_next]]) <= EPS) {
                    if (ptInTriInc(poly[idx[k]], a, b, c)) { ear = false; break; }
                }
            }
            if (ear) { tris.push_back({ a,b,c }); idx.erase(idx.begin() + ii); found = true; break; }
        }
        if (!found)break;
    }
    if ((int)idx.size() == 3)tris.push_back({ poly[idx[0]],poly[idx[1]],poly[idx[2]] });
    return tris;
}

// ---- Ray-segment intersection ----
inline double raySegT(double ox, double oy, double dx, double dy,
    double ax, double ay, double bx, double by) {
    double ex = bx - ax, ey = by - ay, den = dx * ey - dy * ex;
    if (fabs(den) < EPS)return-1;
    double fx = ax - ox, fy = ay - oy;
    double t = (fx * ey - fy * ex) / den, s = (fx * dy - fy * dx) / den;
    return(t > EPS && s >= -EPS && s <= 1.0 + EPS) ? t : -1;
}

// ============================================================
// NFP data with bounding boxes for fast rejection
// ============================================================
struct BBox { double xmin, ymin, xmax, ymax; };

struct NFPPart {
    Poly verts;
    BBox bb;
    void computeBB() {
        bb = { 1e18,1e18,-1e18,-1e18 };
        for (auto& v : verts) {
            if (v.x < bb.xmin)bb.xmin = v.x; if (v.x > bb.xmax)bb.xmax = v.x;
            if (v.y < bb.ymin)bb.ymin = v.y; if (v.y > bb.ymax)bb.ymax = v.y;
        }
    }
    inline bool bbContains(double px, double py) const {
        return px >= bb.xmin - EPS && px <= bb.xmax + EPS && py >= bb.ymin - EPS && py <= bb.ymax + EPS;
    }
};

struct NFPData {
    vector<NFPPart> parts;
    vector<pair<double, double>> baseDirs;
    bool bothConvex;
    Poly nfp;        
    V refPt;         
    BBox globalBB;   
};

// ---- NFP construction ----
void computeNFPParts(const Poly& A, const Poly& B, NFPData& data) {
    int nb = B.size();
    V ref = B[0]; 
    Poly negB(nb);
    for (int i = 0; i < nb; i++)negB[i] = V(ref.x - B[i].x, ref.y - B[i].y);
    ensureCCW(negB);
    auto tA = triangulate(A);
    auto tNB = triangulate(negB);
    for (auto& ta : tA) {
        Poly t = ta; ensureCCW(t);
        for (auto& tb : tNB) {
            Poly t2 = tb; ensureCCW(t2);
            Poly ms = minkowskiSum(t, t2);
            if ((int)ms.size() >= 3) {
                NFPPart np; np.verts = ms; np.computeBB();
                data.parts.push_back(np);
            }
        }
    }
    data.globalBB = { 1e18,1e18,-1e18,-1e18 };
    for (auto& p : data.parts) {
        if (p.bb.xmin < data.globalBB.xmin)data.globalBB.xmin = p.bb.xmin;
        if (p.bb.ymin < data.globalBB.ymin)data.globalBB.ymin = p.bb.ymin;
        if (p.bb.xmax > data.globalBB.xmax)data.globalBB.xmax = p.bb.xmax;
        if (p.bb.ymax > data.globalBB.ymax)data.globalBB.ymax = p.bb.ymax;
    }
}

void precomputeDirs(NFPData& data) {
    struct DK { long long a, b; bool operator<(const DK& o)const { return a != o.a ? a < o.a : b < o.b; } };
    vector<DK> keys;
    vector<pair<double, double>> dirs;
    const double SC = 1e8;

    for (auto& np : data.parts) {
        auto& p = np.verts; int n = p.size();
        for (int i = 0; i < n; i++) {
            V edge = p[(i + 1) % n] - p[i];
            V nm = edge.perp().unit();
            if (nm.norm() < EPS)continue;
            keys.push_back({ (long long)round(nm.x * SC),(long long)round(nm.y * SC) });
            dirs.push_back({ nm.x,nm.y });
            keys.push_back({ (long long)round(-nm.x * SC),(long long)round(-nm.y * SC) });
            dirs.push_back({ -nm.x,-nm.y });
        }
    }
    vector<int> order(keys.size());
    for (int i = 0; i < (int)order.size(); i++)order[i] = i;
    sort(order.begin(), order.end(), [&](int a, int b) {return keys[a] < keys[b]; });
    for (int i = 0; i < (int)order.size(); i++) {
        if (i > 0 && !(keys[order[i - 1]] < keys[order[i]]))continue;
        data.baseDirs.push_back(dirs[order[i]]);
    }
}

// ---- Solve convex ----
V solveConvex(V query, const Poly& nfp) {
    if (pip(query, nfp) <= 0)return{ 0,0 };
    double minD2 = DBL_MAX; V bestCP;
    int n = nfp.size();
    for (int i = 0; i < n; i++) { V cp; double d2 = closestOnSeg(query, nfp[i], nfp[(i + 1) % n], cp); if (d2 < minD2) { minD2 = d2; bestCP = cp; } }
    return bestCP - query;
}

// ---- Solve general with bbox optimization ----
V solveGeneral(V query, const NFPData& data) {
    double qx = query.x, qy = query.y;

    if (qx<data.globalBB.xmin - EPS || qx>data.globalBB.xmax + EPS ||
        qy<data.globalBB.ymin - EPS || qy>data.globalBB.ymax + EPS)
        return{ 0,0 };

    static vector<int> containing;
    containing.clear();
    int np = data.parts.size();
    for (int i = 0; i < np; i++) {
        if (!data.parts[i].bbContains(qx, qy))continue;
        if (pip(qx, qy, data.parts[i].verts.data(), data.parts[i].verts.size()) > 0)
            containing.push_back(i);
    }
    if (containing.empty())return{ 0,0 };

    struct DK { long long a, b; bool operator<(const DK& o)const { return a != o.a ? a < o.a : b < o.b; } };
    static vector<DK> seenKeys;
    static vector<pair<double, double>> dirs;
    seenKeys.clear(); dirs.clear();
    const double SC = 1e8;

    for (auto& d : data.baseDirs) {
        seenKeys.push_back({ (long long)round(d.first * SC),(long long)round(d.second * SC) });
        dirs.push_back(d);
    }
    for (int idx : containing) {
        for (auto& v : data.parts[idx].verts) {
            double dx = v.x - qx, dy = v.y - qy;
            double ln = sqrt(dx * dx + dy * dy);
            if (ln < EPS)continue;
            dx /= ln; dy /= ln;
            seenKeys.push_back({ (long long)round(dx * SC),(long long)round(dy * SC) });
            dirs.push_back({ dx,dy });
        }
    }
    
    int nd = dirs.size();
    static vector<int> order;
    order.resize(nd);
    for (int i = 0; i < nd; i++)order[i] = i;
    sort(order.begin(), order.end(), [&](int a, int b) {return seenKeys[a] < seenKeys[b]; });

    double bestT = DBL_MAX, bestDx = 0, bestDy = 0;
    int prevOrd = -1;
    
    for (int oi = 0; oi < nd; oi++) {
        int ci = order[oi];
        if (prevOrd >= 0 && !(seenKeys[order[prevOrd]] < seenKeys[ci])) { continue; }
        prevOrd = oi;

        double dx = dirs[ci].first, dy = dirs[ci].second;
        double maxExitT = 0; bool valid = true;
        for (int idx : containing) {
            auto& pv = data.parts[idx].verts; int pn = pv.size();
            double minT = DBL_MAX;
            for (int i = 0; i < pn; i++) {
                int j = (i + 1) % pn;
                double t = raySegT(qx, qy, dx, dy, pv[i].x, pv[i].y, pv[j].x, pv[j].y);
                if (t > EPS && t < minT)minT = t;
            }
            if (minT >= DBL_MAX / 2) { valid = false; break; }
            if (minT > maxExitT)maxExitT = minT;
        }
        if (!valid || maxExitT < EPS)continue;
        if (maxExitT >= bestT)continue;

        // 使用 1e-6 作为安全边界检测补偿值
        double step = 1e-6; 
        double mx = qx + dx * (maxExitT + step), my = qy + dy * (maxExitT + step);
        bool allOut = true;
        for (int i = 0; i < np; i++) {
            if (!data.parts[i].bbContains(mx, my))continue;
            if (pip(mx, my, data.parts[i].verts.data(), data.parts[i].verts.size()) > 0) { allOut = false; break; }
        }

        if (allOut) {
            bestT = maxExitT; bestDx = dx * maxExitT; bestDy = dy * maxExitT;
        }
        else {
            double tc = maxExitT;
            for (int iter = 0; iter < 100; iter++) { // 加强探测上限，应对密集复杂 NFP 集合
                double mx2 = qx + dx * (tc + step), my2 = qy + dy * (tc + step);
                static vector<int> nc; nc.clear();
                for (int i = 0; i < np; i++) {
                    if (!data.parts[i].bbContains(mx2, my2))continue;
                    if (pip(mx2, my2, data.parts[i].verts.data(), data.parts[i].verts.size()) > 0)nc.push_back(i);
                }
                if (nc.empty()) { if (tc < bestT) { bestT = tc; bestDx = dx * tc; bestDy = dy * tc; }break; }
                double newMax = tc; bool fail = false;
                for (int idx2 : nc) {
                    auto& pv = data.parts[idx2].verts; int pn = pv.size();
                    double minT = DBL_MAX;
                    for (int i = 0; i < pn; i++) {
                        int j = (i + 1) % pn;
                        double t = raySegT(qx, qy, dx, dy, pv[i].x, pv[i].y, pv[j].x, pv[j].y);
                        if (t > tc + EPS && t < minT)minT = t;
                    }
                    if (minT >= DBL_MAX / 2) { fail = true; break; }
                    if (minT > newMax)newMax = minT;
                }
                if (fail || newMax <= tc + EPS)break;
                tc = newMax;
            }
        }
    }
    return{ bestDx,bestDy };
}

// ============================================================
// Main I/O
// ============================================================
int main() {
    int n1, n2;
    if (scanf("%d%d", &n1, &n2) != 2) return 0;
    Poly polyA(n1), polyB(n2);
    for (int i = 0; i < n1; i++)scanf("%lf%lf", &polyA[i].x, &polyA[i].y);
    for (int i = 0; i < n2; i++)scanf("%lf%lf", &polyB[i].x, &polyB[i].y);
    char buf[64]; scanf("%s", buf);// "OK"

    ensureCCW(polyA); ensureCCW(polyB);
    NFPData data;
    data.refPt = polyB[0];
    data.bothConvex = isConvex(polyA) && isConvex(polyB);

    if (data.bothConvex) {
        V ref = polyB[0];
        Poly negB(n2);
        for (int i = 0; i < n2; i++)negB[i] = V(ref.x - polyB[i].x, ref.y - polyB[i].y);
        ensureCCW(negB);
        data.nfp = minkowskiSum(polyA, negB);
    }
    else {
        computeNFPParts(polyA, polyB, data);
        precomputeDirs(data);
    }

    printf("OK\n"); fflush(stdout);

    int m; 
    if (scanf("%d", &m) != 1) return 0;
    vector<V> tests(m);
    for (int i = 0; i < m; i++)scanf("%lf%lf", &tests[i].x, &tests[i].y);
    scanf("%s", buf);// "OK"

    //printf("%d\n", m);
    for (int i = 0; i < m; i++) {
        V query = data.refPt + tests[i];
        V res;
        if (data.bothConvex) res = solveConvex(query, data.nfp);
        else res = solveGeneral(query, data);
        
        double rx = res.x, ry = res.y;
        
        // 保留5位小数严格防患 -0.00000 现象
        rx = round(rx * 100000.0) / 100000.0;
        ry = round(ry * 100000.0) / 100000.0;
        
        if (fabs(rx) < 1e-9) rx = 0.0;
        if (fabs(ry) < 1e-9) ry = 0.0;
        
        printf("%.5f %.5f\n", rx, ry);
    }
    fflush(stdout);
    printf("OK\n"); fflush(stdout);
    return 0;
}