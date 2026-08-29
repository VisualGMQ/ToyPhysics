#include "toy_physics/gjk.hpp"

#include "toy_physics/algorithm.hpp"
#include "toy_physics/check.hpp"
#include "toy_physics/structure.hpp"

namespace toy_physics {

bool MTD::IsValid() const {
    return *this != null_mtd;
}

null_mtd_t::operator bool() const {
    return false;
}

null_mtd_t::operator MTD() const {
    return MTD{Vector3::Zero(), -1};
}

bool null_mtd_t::operator==(null_mtd_t) const {
    return true;
}

bool null_mtd_t::operator!=(null_mtd_t) const {
    return false;
}

bool null_mtd_t::operator==(const MTD& o) const {
    return o.m_len < 0;
}

bool null_mtd_t::operator!=(const MTD& o) const {
    return !(*this == o);
}

bool operator==(const MTD& o, null_mtd_t) {
    return o.m_len < 0;
}

bool operator!=(const MTD& o, null_mtd_t null) {
    return !(o == null);
}

bool CalcCommonPoint(const SupportFunction& support1,
                     const SupportFunction& support2, Vector3 init_dir,
                     Vector3& out_p1, Vector3& out_p2, real tolerance) {
    GJK gjk;

    real dist2 = std::numeric_limits<real>::max();
    Vector3 v = init_dir;
    Vector3 p1, p2, w;
    real prev_dist2;

    do {
        p1 = support1(-v);
        p2 = support2(v);
        w = p1 - p2;

        if (v.dot(w) > 0) {
            return false;
        }
        gjk.AddVertex(w, p1, p2);

        if (gjk.IsAffinelyDependent()) {
            return false;
        }

        if (!gjk.GetClosest(v)) {
            return false;
        }

        prev_dist2 = dist2;

        dist2 = v.squaredNorm();

        if (prev_dist2 - dist2 <= kREAL_EPSILON * prev_dist2) {
            return false;
        }
    } while (!gjk.IsFullSimplex() && dist2 > tolerance * gjk.GetMaxDist2());

    gjk.ComputeNearestPoints(out_p1, out_p2);
    return true;
}

real CalcClosestPoints(const SupportFunction& support1,
                       const SupportFunction& support2, Vector3& out_p1,
                       Vector3& out_p2, real max_dist2, real tolerance) {
    Vector3 v = Vector3::UnitX();
    GJK gjk;

    real dist2 = kREAL_MAX;

    do {
        Vector3 p = support1(-v);
        Vector3 q = support2(v);
        Vector3 w = p - q;

        real delta = v.dot(w);
        if (delta > 0 && delta * delta > dist2 * max_dist2) {
            return kREAL_MAX;
        }

        if (gjk.IsInSimplex(w) ||
            dist2 - delta <= dist2 * tolerance * tolerance) {
            break;
        }

        gjk.AddVertex(w, p, q);
        if (gjk.IsAffinelyDependent()) {
            break;
        }

        if (!gjk.GetClosest(v)) {
            break;
        }

        real prev_dist2 = dist2;

        dist2 = v.squaredNorm();

        if (prev_dist2 - dist2 <= kREAL_EPSILON * prev_dist2) {
            gjk.GetBackupClosest(v);
            dist2 = v.squaredNorm();
            break;
        }
    } while (!gjk.IsFullSimplex() && dist2 > tolerance * gjk.GetMaxDist2());

    assert(!gjk.IsEmptySimplex());

    if (dist2 <= max_dist2) {
        gjk.ComputeNearestPoints(out_p1, out_p2);
    }

    return dist2;
}

[[nodiscard]] bool IsIntersect(const SupportFunction& support1,
                               const SupportFunction& support2,
                               Vector3 init_dir, real tolerance) {
    GJK gjk;
    real dist2 = kREAL_MAX;
    Vector3 v = init_dir;

    do {
        Vector3 p = support1(-v);
        Vector3 q = support2(v);
        Vector3 w = p - q;

        if (v.dot(w) > 0) {
            return false;
        }

        gjk.AddVertex(w);
        if (gjk.IsAffinelyDependent()) {
            return false;
        }

        if (!gjk.GetClosest(v)) {
            return false;
        }

        real prev_dist2 = dist2;

        dist2 = v.squaredNorm();

        if (prev_dist2 - dist2 <= kREAL_EPSILON * prev_dist2) {
            return false;
        }
    } while (!gjk.IsFullSimplex() && dist2 > tolerance * gjk.GetMaxDist2());

    return true;
}

int Sweep(const SupportFunction& support1, const SupportFunction& support2,
          Vector3 dir, real len, bool need_mtd, real* out_t,
          Vector3* out_position, Vector3* out_normal, Vector3* out_witness1,
          Vector3* out_witness2, real tolerance) {
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len >= kREAL_EPSILON, 0);

    Vector3 r = dir * len;

    real lambda = 0;
    Vector3 x = Vector3::Zero();
    Vector3 n = Vector3::Zero();
    Vector3 v = support1(-dir) - support2(dir);
    Vector3 p1 = Vector3::Zero();
    Vector3 p2 = Vector3::Zero();
    GJK gjk;

    real max_dist2 = 0;
    while (v.squaredNorm() > tolerance * max_dist2) {
        Vector3 s2 = support2(v);
        Vector3 s1 = support1(-v);
        Vector3 p = s2 - s1;
        Vector3 w = x - p;
        max_dist2 = std::max(max_dist2, w.squaredNorm());
        if (v.dot(w) > 0) {
            if (v.dot(r) >= 0) {
                return 0;
            }
            lambda -= v.dot(w) / v.dot(r);
            if (lambda > 1) {
                return 0;
            }
            x = lambda * r;
            n = v;
        }
        // numerical degeneracy (tiny Delta_i(Y))
        // result instead of adding to a full simplex
        if (gjk.IsFullSimplex()) {
            break;
        }
        if (gjk.IsInSimplex(w)) {
            break;
        }
        gjk.AddVertex(w, x + s1, s2);
        if (!gjk.GetClosest(v)) {
            v = Vector3::Zero();
        }
    }

    if (need_mtd && lambda <= 0 && !gjk.IsEmptySimplex()) {
        gjk.ComputeNearestPoints(p1, p2);
    }

    // initial overlap
    if (lambda <= 0) {
        if (out_t) {
            *out_t = 0;
        }
        if (out_position) {
            *out_position = x;
        }
        if (out_normal) {
            *out_normal = Vector3::Zero();
        }
        if (need_mtd) {
            if (out_witness1) {
                *out_witness1 = p1;
            }
            if (out_witness2) {
                *out_witness2 = p2;
            }
        }
        return -1;
    }

    if (out_t) {
        *out_t = lambda;
    }
    if (out_position) {
        *out_position = x;
    }
    if (out_normal) {
        *out_normal = n.isZero() ? Vector3::Zero() : n.normalized();
    }
    return 1;
}

bool GJK::IsEmptySimplex() const {
    return m_bits == 0x0;
}

bool GJK::IsFullSimplex() const {
    return m_bits == 0xF;
}

real GJK::GetMaxDist2() const {
    return m_max_len2;
}

void GJK::AddVertex(Vector3 w, Vector3 p1, Vector3 p2) {
    AddVertex(w);
    m_p1_list[m_last] = p1;
    m_p2_list[m_last] = p2;
}

void GJK::AddVertex(Vector3 w) {
    TOY_ASSERT(!IsFullSimplex());
    m_last = 0;
    m_last_bit = 0x1;

    while (contain(m_last_bit, m_bits)) {
        ++m_last;
        m_last_bit <<= 1;
    }

    m_y_list[m_last] = w;
    m_y_len2[m_last] = w.squaredNorm();
    m_all_bits = m_bits | m_last_bit;

    updateCache();
    computeDet();
}

void GJK::updateCache() {
    uint8_t bit = 0x1;
    for (uint8_t i = 0; i < 4; i++) {
        if (contain(m_bits, bit)) {
            m_edges[i][m_last] = m_y_list[i] - m_y_list[m_last];
            m_edges[m_last][i] = -m_edges[i][m_last];
        }
        bit <<= 1;
    }
}

bool GJK::contain(uint8_t src_bits, uint8_t target_bits) const {
    return src_bits & target_bits;
}

bool GJK::proper(uint8_t s) const {
    int i;
    uint8_t bit;
    for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) {
        if (contain(s, bit) && m_dets[s][i] <= 0) {
            return false;
        }
    }
    return true;
}

void GJK::computeDet() {
    m_dets[m_last_bit][m_last] = 1;

    if (m_bits == 0) {
        return;
    }

    for (uint8_t i = 0, si = 0x1; i < 4; i++, si <<= 1) {
        if (contain(m_bits, si)) {
            uint8_t s2 = si | m_last_bit;
            m_dets[s2][i] = m_edges[m_last][i].dot(m_y_list[m_last]);
            m_dets[s2][m_last] = m_edges[i][m_last].dot(m_y_list[i]);

            for (uint8_t j = 0, sj = 0x1; j < i; ++j, sj <<= 1) {
                if (contain(m_bits, sj)) {
                    uint8_t s3 = sj | s2;
                    m_dets[s3][j] =
                        m_dets[s2][i] * m_edges[i][j].dot(m_y_list[i]) +
                        m_dets[s2][m_last] *
                            m_edges[i][j].dot(m_y_list[m_last]);
                    m_dets[s3][i] = m_dets[sj | m_last_bit][j] *
                                        m_edges[j][i].dot(m_y_list[j]) +
                                    m_dets[sj | m_last_bit][m_last] *
                                        m_edges[j][i].dot(m_y_list[m_last]);
                    m_dets[s3][m_last] =
                        m_dets[sj | si][j] *
                            m_edges[j][m_last].dot(m_y_list[j]) +
                        m_dets[sj | si][i] *
                            m_edges[j][m_last].dot(m_y_list[i]);
                }
            }
        }
    }

    if (m_all_bits == 0xf) {
        m_dets[0xf][0] = m_dets[0xe][1] * m_edges[1][0].dot(m_y_list[1]) +
                         m_dets[0xe][2] * m_edges[1][0].dot(m_y_list[2]) +
                         m_dets[0xe][3] * m_edges[1][0].dot(m_y_list[3]);
        m_dets[0xf][1] = m_dets[0xd][0] * m_edges[0][1].dot(m_y_list[0]) +
                         m_dets[0xd][2] * m_edges[0][1].dot(m_y_list[2]) +
                         m_dets[0xd][3] * m_edges[0][1].dot(m_y_list[3]);
        m_dets[0xf][2] = m_dets[0xb][0] * m_edges[0][2].dot(m_y_list[0]) +
                         m_dets[0xb][1] * m_edges[0][2].dot(m_y_list[1]) +
                         m_dets[0xb][3] * m_edges[0][2].dot(m_y_list[3]);
        m_dets[0xf][3] = m_dets[0x7][0] * m_edges[0][3].dot(m_y_list[0]) +
                         m_dets[0x7][1] * m_edges[0][3].dot(m_y_list[1]) +
                         m_dets[0x7][2] * m_edges[0][3].dot(m_y_list[2]);
    }
}

bool GJK::isSubseqEQ(uint8_t a, uint8_t b) const {
    return (a & b) == a;
}

bool GJK::isSetValid(uint8_t s) const {
    for (uint8_t i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) {
        if (contain(m_all_bits, bit)) {
            if (contain(s, bit)) {
                // check Delta_i(X) > 0
                if (m_dets[s][i] <= 0) {
                    return false;
                }
                // check Delta_i(X \cup {y_i}) < 0
            } else if (m_dets[s | bit][i] > 0) {
                return false;
            }
        }
    }
    return true;
}

Vector3 GJK::computeVector(uint8_t s) {
    Vector3 v = Vector3::Zero();
    m_max_len2 = 0;

    real sum = 0;  // the Delta: common denominator by Cramer ruler

    for (uint8_t i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) {
        if (contain(s, bit)) {
            /**
             * nearest point is:
             * lambda_i * y_i = \sum{y_i * Delta_i/Delta}
             *                = 1/Delta * \sum{y_i * Delta_i}
             *
             * here we calc molecule and denominator separatly:
             * sum = Delta = \sum{Delta_i}
             * v = \sum{y_i * Delta_i}
             */
            sum += m_dets[s][i];
            m_max_len2 = std::max(m_max_len2, m_y_len2[i]);
            v += m_y_list[i] * m_dets[s][i];
        }
    }

    ASSERT(sum > 0, "sum must > 0");
    // finally, nearest point = v / sum
    return v / sum;
}

void GJK::ComputeNearestPoints(Vector3& out_p1, Vector3& out_p2) const {
    real sum = 0;
    out_p1 = Vector3::Zero();
    out_p2 = Vector3::Zero();
    for (uint8_t i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) {
        if (contain(m_bits, bit)) {
            sum += m_dets[m_bits][i];
            out_p1 += m_p1_list[i] * m_dets[m_bits][i];
            out_p2 += m_p2_list[i] * m_dets[m_bits][i];
        }
    }

    ASSERT(sum > 0, "sum must > 0");
    const real s = 1.0 / sum;
    out_p1 *= s;
    out_p2 *= s;
}

bool GJK::IsInSimplex(Vector3 w) const {
    int i;
    uint8_t bit;
    for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) {
        if (contain(m_all_bits, bit) && w == m_y_list[i]) {
            return true;
        }
    }
    return false;
}

void GJK::GetBackupClosest(Vector3& v) {
    real min_dist2 = kREAL_MAX;

    for (uint8_t s = m_all_bits; s != 0x0; --s) {
        if (isSubseqEQ(s, m_all_bits) && proper(s)) {
            Vector3 u = computeVector(s);
            real dist2 = u.squaredNorm();
            if (dist2 < min_dist2) {
                min_dist2 = dist2;
                m_bits = s;
                v = u;
            }
        }
    }
}

int GJK::GetSimplex(std::span<Vector3> p_buf, std::span<Vector3> q_buf,
                    std::span<Vector3> y_buf) const {
    TOY_ASSERT(p_buf.size() == q_buf.size() && p_buf.size() == y_buf.size());

    int num_verts = 0;
    for (uint8_t i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) {
        if (contain(m_bits, bit)) {
            p_buf[num_verts] = m_p1_list[i];
            q_buf[num_verts] = m_p2_list[i];
            y_buf[num_verts] = m_y_list[i];

            ++num_verts;
        }
    }
    return num_verts;
}

bool GJK::IsAffinelyDependent() const {
    real sum = 0;
    for (uint8_t i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
        if (contain(m_all_bits, bit)) {
            sum += m_dets[m_all_bits][i];
        }
    }
    return sum <= 0;
}

bool GJK::GetClosest(Vector3& v) {
    // Fast closest. Only find set contains last simplex vertex
    for (uint8_t s = m_bits; s != 0x0; --s) {
        if (isSubseqEQ(s, m_bits) && isSetValid(s | m_last_bit)) {
            m_bits = s | m_last_bit;
            v = computeVector(m_bits);
            return true;
        }
    }
    if (isSetValid(m_last_bit)) {
        m_bits = m_last_bit;
        m_max_len2 = m_y_len2[m_last];
        v = m_y_list[m_last];
        return true;
    }
    return false;
}

}  // namespace toy_physics
