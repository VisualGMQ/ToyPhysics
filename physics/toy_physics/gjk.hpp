#pragma once

#include "check.hpp"
#include "toy_physics/math.hpp"
#include "toy_physics/structure.hpp"

namespace toy_physics {

struct MTD {
    Vector3 m_dir{Vector3::Zero()};
    real m_len{-1};

    [[nodiscard]] bool IsValid() const;
};

struct null_mtd_t {
    operator bool() const;
    operator MTD() const;
    bool operator==(null_mtd_t) const;
    bool operator!=(null_mtd_t) const;
    bool operator==(const MTD& o) const;
    bool operator!=(const MTD& o) const;
};

bool operator==(const MTD& o, null_mtd_t);
bool operator!=(const MTD& o, null_mtd_t null);

static constexpr null_mtd_t null_mtd{};

struct SupportFunction {
    virtual ~SupportFunction() = default;
    [[nodiscard]] virtual Vector3 operator()(Vector3 dir) const = 0;
};

class GJK {
public:
    /**
     * check simplex is affinely dependent by check \f$ \Delta(X) \le 0 \f$ from
     * \f$ \lambda_i = \frac{\Delta_i(X)}{\Delta(X)} \f$
     */
    [[nodiscard]] bool IsAffinelyDependent() const;
    [[nodiscard]] bool GetClosest(Vector3&);

    /**
     * add a minkowsi diff vertex and it's original vertices.
     * @param w  minkowsi diff(p1 - p2)
     * @param p1 original vertex on shape1
     * @param p2 original vertex on shape2
     */
    void AddVertex(Vector3 w, Vector3 p1, Vector3 p2);

    /**
     * add w to simplex and recompute det
     * @param w minkowsi diff
     */
    void AddVertex(Vector3 w);

    [[nodiscard]] bool IsEmptySimplex() const;
    [[nodiscard]] bool IsFullSimplex() const;

    /**
     * get square of max distance from support point to origin
     */
    [[nodiscard]] real GetMaxDist2() const;

    /**
     * compute corresponding point from shape1, shape2
     * @param out_p1 from shape1
     * @param out_p2 from shape2
     */
    void ComputeNearestPoints(Vector3& out_p1, Vector3& out_p2) const;

    [[nodiscard]] bool IsInSimplex(Vector3 w) const;

    /**
     *  backup procedure in GJK 1988
     */
    void GetBackupClosest(Vector3& v);

    [[nodiscard]] int GetSimplex(std::span<Vector3> p_buf,
                                 std::span<Vector3> q_buf,
                                 std::span<Vector3> y_buf) const;

private:
    /**
     * det for speed up new det calculate(
     * for \f$ \Delta_i(Y) \f$ and \f$ \Delta(Y) \f$)
     *
     * directly from GinoGJK lecture
     */
    std::array<std::array<real, 4>, 16> m_dets{};

    /**
     * edge vector in simplex(+edge & -edge).
     *
     * correspond to \f$ (y_k - y_j) \f$ in
     * \f$ \Delta_j(Y_s \cup \{y_i\}) = \sum_{i \in I} \Delta_i(Y_s)(y_k - y_j)
     * \cdot y_i \f$
     */
    std::array<std::array<Vector3, 4>, 4> m_edges;

    uint8_t m_bits{0};      ///< all bit contains simplex vertices(\f$ W_k \f$)
    uint8_t m_all_bits{0};  ///< all bit contains simplex vertices and new
                            ///< vertex(\f$ W_k \cup \vec{w_k} \f$)
    uint8_t m_last{0};      ///< last free vertex slot
    uint8_t m_last_bit{0};  ///< m_last correspond bit. @see m_last

    real m_dist{0};
    Vector3 m_nearest_p1;
    Vector3 m_nearest_p2;
    std::array<Vector3, 4> m_p1_list;
    std::array<Vector3, 4> m_p2_list;
    std::array<Vector3, 4> m_y_list;  ///< vertex in simplex
    std::array<real, 4>
        m_y_len2{};  ///< vertex length squared in simplex, @see m_y_list
    real m_max_len2{0};

    /**
     * update m_edges when new point add to simplex
     */
    void updateCache();

    /**
     * check whether contain src_bits referred point
     */
    [[nodiscard]] bool contain(uint8_t src_bits, uint8_t target_bits) const;

    [[nodiscard]] bool proper(uint8_t s) const;

    /**
     * compute det for new simplex vertex.
     *
     */
    void computeDet();

    /**
     * check \f$ a \subseteq b \f$
     */
    [[nodiscard]] bool isSubseqEQ(uint8_t a, uint8_t b) const;

    /**
     * check set \f$ \Delta_i(X) \ge 0 \f$ and \f$ \Delta(X) \ge 0 \f$
     */
    [[nodiscard]] bool isSetValid(uint8_t set) const;

    /**
     * compute the point nearest to origin
     */
    [[nodiscard]] Vector3 computeVector(uint8_t s);
};

[[nodiscard]] bool CalcCommonPoint(const SupportFunction& support1,
                                   const SupportFunction& support2,
                                   Vector3 init_dir, Vector3& out_p1,
                                   Vector3& out_p2, real tolerance = 1e-3);
real CalcClosestPoints(const SupportFunction& support1,
                       const SupportFunction& support2, Vector3& out_p1,
                       Vector3& out_p2, real max_dist2, real tolerance = 1e-3);
[[nodiscard]] bool IsIntersect(const SupportFunction& support1,
                               const SupportFunction& support2,
                               Vector3 init_dir, real tolerance = 1e-3);
}  // namespace toy_physics
