// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Array collision kernels, written once against murosim::simd::Vec.
// The build-selected SIMD backend (AVX-512, AVX, SSE, NEON or scalar) is
// chosen in vec.hpp; ArrayFloat pads to whole vectors with quiet NaN, which
// the ordered comparisons below treat as "no collision".

#include "murosim/simd/vec.hpp"
#include "murosim/simd/array_float.hpp"

#include "murosim/collision/circle.hpp"
#include "murosim/collision/laser.hpp"
#include "murosim/collision/line.hpp"

#include <algorithm>

namespace
{

using Vec = murosim::simd::Vec;
using V = Vec::Type;
using M = Vec::Mask;

// ArrayFloat pads its storage to whole SIMD_SIZE-lane blocks. The kernels load
// Vec::LANES floats at a time, so a padding block must be at least a full
// vector or the final Vec::load would read past the buffer.
static_assert(murosim::simd::ArrayFloat::SIMD_SIZE == Vec::LANES,
              "SIMD array padding must match the kernel lane count");

constexpr float EPSILON = 0.0001f;

// widen an index window to whole SIMD lanes
size_t alignDown(const size_t index)
{
    return (index / Vec::LANES) * Vec::LANES;
}

size_t alignUp(const size_t index)
{
    return ((index + Vec::LANES - 1) / Vec::LANES) * Vec::LANES;
}

// Store `value` across the lane covering array[i .. i+LANES), but only the
// elements whose index lies in [lo, hi). A fully-in-window lane takes the plain
// vector store (the hot path -- full scans pass [0, sizeAligned) and always hit
// it); the at-most-two partial boundary lanes copy element-wise, so rays just
// outside the window are never touched and callers can pass an exact range.
void storeWindow(float *array, const size_t i, const V value, const size_t lo, const size_t hi)
{
    if (lo <= i && i + Vec::LANES <= hi)
    {
        Vec::store(array + i, value);
        return;
    }

    alignas(64) float lane[Vec::LANES];
    Vec::store(lane, value);

    const size_t begin = std::max(i, lo);
    const size_t end = std::min(i + Vec::LANES, hi);
    for (size_t k = begin; k < end; k++)
    {
        array[k] = lane[k - i];
    }
}

/**
 * Solid-disk test for segment lanes against circle lanes: a segment collides
 * if it crosses the circle boundary OR an endpoint lies inside the disk
 * (which also catches a segment fully contained in the circle).
 *
 * Parametrise the segment as P(t) = start + t * d for t in [0, 1], where d is
 * its direction, f the vector from the circle centre to the start and g the
 * same for the end. Substituting into |P(t) - centre|^2 = r^2 gives a quadratic
 * in t:
 *
 *   (d.d) t^2 + 2 (f.d) t + (f.f - r^2) = 0
 *
 * so a = d.d, b = 2 (f.d), c = f.f - r^2, with roots (-b -+ sqrt(b^2 - 4ac))/2a.
 * The boundary is crossed within the segment when the discriminant is
 * non-negative and a root falls inside [0, 1]; g serves the endpoint test.
 *
 * A degenerate point segment yields t = NaN in the crossing test, but the
 * endpoint test still reports a point inside the disk -- matching the scalar
 * lineWithCircle. Single owner of this math for lineWithCircleArray and
 * circleWithLineArray, so the two directions can never diverge.
 */
M segmentWithDiskMask(const V dx, const V dy, const V fx, const V fy, const V gx, const V gy, const V r_sq)
{
    const V two = Vec::set1(2.0f);
    const V four = Vec::set1(4.0f);
    const V zero = Vec::zero();
    const V one = Vec::set1(1.0f);

    // a = d.Dot(d)
    const V a = dx*dx + dy*dy;

    const V f_sq = fx*fx + fy*fy; // |E - C|^2, reused by the endpoint test

    // b = 2*f.Dot( d )
    const V b = two * (fx*dx + fy*dy);

    // c = f.Dot( f ) - r*r
    const V c = f_sq - r_sq;

    // discriminant = sqrt(b*b - 4*a*c)
    const V discriminant = Vec::sqrt(b*b - four*a*c);

    const V two_a = two*a;

    // t1 = (-b - discriminant)/(2*a)
    const V t1 = Vec::div(-b - discriminant, two_a);

    // t2 = (-b + discriminant)/(2*a)
    const V t2 = Vec::div(discriminant - b, two_a);

    // boundary crossing: (0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1)
    const M crossing = Vec::maskOr(
        Vec::maskAnd(Vec::lessEqual(zero, t1), Vec::lessEqual(t1, one)),
        Vec::maskAnd(Vec::lessEqual(zero, t2), Vec::lessEqual(t2, one)));

    // endpoint inside the disk: |E - C|^2 <= r^2 or |L - C|^2 <= r^2
    const M endpoint_inside = Vec::maskOr(
        Vec::lessEqual(f_sq, r_sq),
        Vec::lessEqual(gx*gx + gy*gy, r_sq));

    return Vec::maskOr(crossing, endpoint_inside);
}

} // namespace

namespace murosim::collision
{

bool lineWithCircleArray(const float p1x, const float p1y, const float p2x, const float p2y, const murosim::simd::ArrayFloat& array_cx, const murosim::simd::ArrayFloat& array_cy, const murosim::simd::ArrayFloat& array_cr)
{
    const V x1 = Vec::set1(p1x);
    const V y1 = Vec::set1(p1y);

    const V dx = Vec::set1(p2x) - x1;
    const V dy = Vec::set1(p2y) - y1;

    for (size_t i = 0; i < array_cx.sizeAligned(); i += Vec::LANES)
    {
        const V x0 = Vec::load(array_cx.data() + i);
        const V y0 = Vec::load(array_cy.data() + i);
        const V r0 = Vec::load(array_cr.data() + i);

        // f = E - C; g = L - C = f + d
        const V fx = x1 - x0;
        const V fy = y1 - y0;
        const V gx = fx + dx;
        const V gy = fy + dy;

        if (Vec::anyTrue(segmentWithDiskMask(dx, dy, fx, fy, gx, gy, r0*r0)))
        {
            return true;
        }
    }

    return false;
}

bool lineWithLineArray(const float p1x, const float p1y, const float p2x, const float p2y, const murosim::simd::ArrayFloat& array_p3x, const murosim::simd::ArrayFloat& array_p3y, const murosim::simd::ArrayFloat& array_p4x, const murosim::simd::ArrayFloat& array_p4y)
{
    /**
     * Formular from: Given two points on each line segment.
     * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
     *
     * numerator t: (x1 - x3)(y3 - y4) - (y1 - y3)(x3 - x4)
     * numerator u: (x2 - x1)(y1 - y3) - (y2 - y1)(x1 - x3)
     * denominator: (x1 - x2)(y3 - y4) - (y1 - y2)(x3 - x4)
     */

    const V x1 = Vec::set1(p1x);
    const V y1 = Vec::set1(p1y);
    const V x2 = Vec::set1(p2x);
    const V y2 = Vec::set1(p2y);

    const V zero = Vec::zero();
    const V one = Vec::set1(1.0f);

    for (size_t i = 0; i < array_p3x.sizeAligned(); i += Vec::LANES)
    {
        const V x3 = Vec::load(array_p3x.data() + i);
        const V y3 = Vec::load(array_p3y.data() + i);
        const V x4 = Vec::load(array_p4x.data() + i);
        const V y4 = Vec::load(array_p4y.data() + i);

        const V denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
        const V t = Vec::div((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4), denominator);
        const V u = Vec::div((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3), denominator);

        // (0 <= t && t <= 1) && (0 <= u && u <= 1)
        const M mask = Vec::maskAnd(
            Vec::maskAnd(Vec::lessEqual(zero, t), Vec::lessEqual(t, one)),
            Vec::maskAnd(Vec::lessEqual(zero, u), Vec::lessEqual(u, one)));

        if (Vec::anyTrue(mask))
        {
            return true;
        }
    }

    return false;
}

bool circleWithCircleArray(const float cx, const float cy, const float cr, const murosim::simd::ArrayFloat& array_x, const murosim::simd::ArrayFloat& array_y, const murosim::simd::ArrayFloat& array_r)
{
    const V x1 = Vec::set1(cx);
    const V y1 = Vec::set1(cy);
    const V r1 = Vec::set1(cr);

    for (size_t i = 0; i < array_x.sizeAligned(); i += Vec::LANES)
    {
        const V x2 = Vec::load(array_x.data() + i);
        const V y2 = Vec::load(array_y.data() + i);
        const V r2 = Vec::load(array_r.data() + i);

        const V dx = x2 - x1;
        const V dy = y2 - y1;
        const V r = r2 + r1;

        // dx*dx + dy*dy <= r*r
        if (Vec::anyTrue(Vec::lessEqual(dx*dx + dy*dy, r*r)))
        {
            return true;
        }
    }

    return false;
}

bool circleWithLineArray(const float cx, const float cy, const float cr, const murosim::simd::ArrayFloat& array_x1, const murosim::simd::ArrayFloat& array_y1, const murosim::simd::ArrayFloat& array_x2, const murosim::simd::ArrayFloat& array_y2)
{
    // Same solid-disk test as lineWithCircleArray, with the circle fixed and
    // the lines coming from the arrays.

    const V x0 = Vec::set1(cx);
    const V y0 = Vec::set1(cy);
    const V r0 = Vec::set1(cr);
    const V r0_sq = r0*r0;

    for (size_t i = 0; i < array_x1.sizeAligned(); i += Vec::LANES)
    {
        const V x1 = Vec::load(array_x1.data() + i);
        const V y1 = Vec::load(array_y1.data() + i);
        const V x2 = Vec::load(array_x2.data() + i);
        const V y2 = Vec::load(array_y2.data() + i);

        // f = E - C; g = L - C
        const V fx = x1 - x0;
        const V fy = y1 - y0;
        const V gx = x2 - x0;
        const V gy = y2 - y0;

        if (Vec::anyTrue(segmentWithDiskMask(x2 - x1, y2 - y1, fx, fy, gx, gy, r0_sq)))
        {
            return true;
        }
    }

    return false;
}

void laserWithLine(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float p3x, const float p3y, const float p4x, const float p4y)
{
    laserWithLine(array_x, array_y, array_distance, p3x, p3y, p4x, p4y, 0, array_distance.sizeAligned());
}

void laserWithLine(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float p3x, const float p3y, const float p4x, const float p4y, const size_t index_begin, const size_t index_end)
{
    /**
     * Ray/segment intersection with the lidar in the origin. The ray is
     * P(t) = t * d with the unit direction d, so t is the distance along the
     * ray: no ray length is needed, and array_distance only holds the nearest
     * hit so far (+inf when nothing was hit yet).
     *
     * The segment is Q(u) = p3 + u * e with e = p4 - p3. Solving
     * t * d = p3 + u * e with the 2D cross product a x b = ax*by - ay*bx:
     *
     *   t = (p3 x e) / (d x e)
     *   u = (p3 x d) / (d x e)
     *
     * A hit needs t >= 0 (in front of the lidar) and u in [0, 1] (on the
     * segment). A ray parallel to the segment divides by zero; the resulting
     * inf/NaN fail the ordered comparisons, so it never counts as a hit.
     */

    const V x3 = Vec::set1(p3x);
    const V y3 = Vec::set1(p3y);
    const V ex = Vec::set1(p4x - p3x);
    const V ey = Vec::set1(p4y - p3y);
    const V numerator_t = x3*ey - y3*ex;

    const V zero = Vec::zero();
    const V lower_bound = Vec::set1(0.0f - EPSILON);
    const V upper_bound = Vec::set1(1.0f + EPSILON);

    const size_t end = std::min(alignUp(index_end), array_distance.sizeAligned());

    for (size_t i = alignDown(index_begin); i < end; i += Vec::LANES)
    {
        const V distance_load = Vec::load(array_distance.data() + i);
        const V dx = Vec::load(array_x.data() + i);
        const V dy = Vec::load(array_y.data() + i);

        const V denominator = dx*ey - dy*ex;
        const V t = Vec::div(numerator_t, denominator);
        const V u = Vec::div(x3*dy - y3*dx, denominator);

        // t >= 0 && (0 <= u && u <= 1), with epsilon slack on the segment ends
        const M mask = Vec::maskAnd(Vec::lessEqual(zero, t),
            Vec::maskAnd(Vec::lessEqual(lower_bound, u), Vec::lessEqual(u, upper_bound)));

        const V distance_min = Vec::min(t, distance_load);

        storeWindow(array_distance.data(), i, Vec::blend(distance_load, distance_min, mask), index_begin, index_end);
    }
}

void laserWithCircle(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float cx, const float cy, const float cr)
{
    laserWithCircle(array_x, array_y, array_distance, cx, cy, cr, 0, array_x.sizeAligned());
}

void laserWithCircle(const murosim::simd::ArrayFloat &array_x, const murosim::simd::ArrayFloat &array_y, murosim::simd::ArrayFloat &array_distance, const float cx, const float cy, const float cr, const size_t index_begin, const size_t index_end)
{
    // Ray origin (the lidar) inside the circle: with the solid-disk collision
    // semantics that is a contact, so every ray in the window reads distance 0
    // (below a positive range_min, so the lidar reports it as -inf, too close).
    // The quadratic below cannot report this case (t1 < 0 < t2, the near-root
    // mask never passes), and skipping the sweep is also cheaper. This is one
    // scalar test per circle; the per-ray SIMD loop is unaffected.
    if (cx * cx + cy * cy <= cr * cr)
    {
        for (size_t i = index_begin; i < index_end; i++)
        {
            array_distance.data()[i] = 0.0f;
        }
        return;
    }

    /**
     * Ray P(t) = t * d with the unit direction d against the circle
     * |P - C| = r: substituting gives t^2 - 2 (d.C) t + (C.C - r^2) = 0 (the
     * factor d.d of t^2 is 1), so with b = d.C and c = C.C - r^2 the near hit
     * is t = b - sqrt(b^2 - c), which is also the distance along the ray. No
     * ray length is needed, so array_distance may hold +inf. A ray that misses
     * makes the square root NaN, which fails the comparison below; a circle
     * behind the lidar (b < 0) gives t < 0. c is loop-invariant, since only d
     * varies per ray.
     */

    const V centre_x = Vec::set1(cx);
    const V centre_y = Vec::set1(cy);
    const V c = Vec::set1(cx * cx + cy * cy - cr * cr);
    const V zero = Vec::zero();

    const size_t end = std::min(alignUp(index_end), array_x.sizeAligned());

    for (size_t i = alignDown(index_begin); i < end; i += Vec::LANES)
    {
        const V distance_load = Vec::load(array_distance.data() + i);
        const V dx = Vec::load(array_x.data() + i);
        const V dy = Vec::load(array_y.data() + i);

        const V b = dx*centre_x + dy*centre_y;
        const V t = b - Vec::sqrt(b*b - c);

        // t >= 0: the circle is hit in front of the lidar
        const M mask = Vec::lessEqual(zero, t);

        const V distance_min = Vec::min(t, distance_load);

        storeWindow(array_distance.data(), i, Vec::blend(distance_load, distance_min, mask), index_begin, index_end);
    }
}

} // namespace murosim::collision
