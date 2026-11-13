///|/ Copyright (c) Prusa Research 2016 - 2021 Vojtěch Bubník @bubnikv
///|/
///|/ ported from lib/Slic3r/Fill/Concentric.pm:
///|/ Copyright (c) Prusa Research 2016 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2015 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <algorithm>
#include <cmath>

#include "../ClipperUtils.hpp"
#include "../ShortestPath.hpp"
#include "FillHoneycomb.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Surface.hpp"

namespace Slic3r {

// ldx[修改蜂窝换向填充] Rotate Honeycomb direction by 0/60/120 deg per layer when SameDirectionHoneycomb is false
// so upper single beads lock centered over lower double vertical walls even when fill_angles has only one angle.
std::pair<float, Point> FillHoneycomb::_infill_direction(const Surface *surface) const
{
    std::pair<float, Point> out = Fill::_infill_direction(surface);
    if constexpr (!SameDirectionHoneycomb) {
        if (surface->bridge_angle < 0 && this->layer_id != size_t(-1)) {
            const unsigned int thickness_layers = std::max<unsigned int>(1u, surface->thickness_layers);
            out.first += this->_layer_angle(this->layer_id / thickness_layers);
        }
    }
    return out;
}

void FillHoneycomb::_fill_surface_single(
    const FillParams                &params, 
    unsigned int                     thickness_layers,
    const std::pair<float, Point>   &direction, 
    ExPolygon                        expolygon,
    Polylines                       &polylines_out)
{
    const bool use_custom_spacing = (this->line_spacing > 0.f) && (params.geo_spacing_mode != 0);
    const coord_t min_spacing = coord_t(scale_(this->spacing));
    //ldx[line_spacing] When custom spacing is active, use line_spacing; otherwise fall back to Original density-based spacing.
    coord_t distance = use_custom_spacing ?
        coord_t(scale_(this->line_spacing)) :
        coord_t(min_spacing / params.density);
    if (distance <= 0)
        return;

    if (use_custom_spacing && params.geo_spacing_mode == 2) {
        // ldx[修改蜂窝换向填充] In alternating mode, use consistent object bbox and base angle
        // so Mode 2 derives the exact same cell pitch across 0/60/120 deg layers for interlocking alignment.
        BoundingBox ref_bb = expolygon.contour.bounding_box();
        float ref_angle = direction.first;
        if constexpr (!SameDirectionHoneycomb) {
            if (!empty(this->bounding_box))
                ref_bb = this->bounding_box;
            if (this->angle != FLT_MAX)
                ref_angle = this->angle;
        }
        Polygon bb_poly = ref_bb.polygon();
        bb_poly.rotate(ref_angle, Point(0, 0));
        distance = this->_adjust_solid_spacing(bb_poly.bounding_box().size()(0), distance);
        if (distance <= 0)
            return;
    }

    // cache hexagons math
    CacheID cache_id(distance, min_spacing, use_custom_spacing);
    Cache::iterator it_m = this->cache.find(cache_id);
    if (it_m == this->cache.end()) {
        it_m = this->cache.insert(it_m, std::pair<CacheID, CacheData>(cache_id, CacheData()));
        CacheData &m        = it_m->second;
        m.distance          = cache_id.distance;
        m.hex_side          = coord_t(m.distance / (sqrt(3)/2));
        m.hex_width         = m.distance * 2; // $m->{hex_width} == $m->{hex_side} * sqrt(3);
        coord_t hex_height  = m.hex_side * 2;
        m.pattern_height    = hex_height + m.hex_side;
        m.y_short           = coord_t(m.distance * sqrt(3)/3);
        //ldx[line_spacing] Clamp x_offset so hexagons never invert when line_spacing is smaller than min_spacing.
        m.x_offset          = cache_id.use_custom_spacing ?
            std::min(cache_id.min_spacing / 2, m.distance * 2 / 5) :
            (cache_id.min_spacing / 2);
        m.y_offset          = coord_t(m.x_offset * sqrt(3)/3);
        m.hex_center        = Point(m.hex_width/2, m.hex_side);
    }
    CacheData &m = it_m->second;
    if (m.distance <= 0 || m.hex_side <= 0 || m.y_short + m.hex_side <= 0)
        return;

    const coord_t y_step    = m.y_short + m.hex_side + m.y_short + m.hex_side;
    // ldx[修改蜂窝换向填充] Rotate and grid-align around direction.second (object/part center) instead of m.hex_center near (0,0),
    // placing a regular hexagon center at pivot so both X and Y are centered symmetrically and 0/60/120 deg layers align globally.
    const Point  &pivot     = direction.second;
    const Point   grid_base = pivot - m.hex_center;

    Polylines all_polylines;
    {
        // adjust actual bounding box to the nearest multiple of our hex pattern
        // and align it so that it is centered on pivot and matches across layers
        
        BoundingBox bounding_box = expolygon.contour.bounding_box();
        {
            // rotate bounding box according to infill direction
            Polygon bb_polygon = bounding_box.polygon();
            bb_polygon.rotate(direction.first, pivot);
            bounding_box = bb_polygon.bounding_box();
            
            // extend bounding box so that our pattern will be aligned with other layers
            // subtracting (m.x_offset, m.y_offset) before aligning ensures boundary stubs at min corner are not missed.
            bounding_box.merge(align_to_grid(
                bounding_box.min - Point(m.x_offset, m.y_offset),
                Point(m.hex_width, y_step),
                grid_base));
        }

        coord_t x = bounding_box.min(0);
        while (x <= bounding_box.max(0)) {
            Polyline p;
            coord_t ax[2] = { x + m.x_offset, x + m.distance - m.x_offset };
            for (size_t i = 0; i < 2; ++ i) {
                std::reverse(p.points.begin(), p.points.end()); // turn first half upside down
                for (coord_t y = bounding_box.min(1); y <= bounding_box.max(1); y += y_step) {
                    p.points.push_back(Point(ax[1], y + m.y_offset));
                    p.points.push_back(Point(ax[0], y + m.y_short - m.y_offset));
                    p.points.push_back(Point(ax[0], y + m.y_short + m.hex_side + m.y_offset));
                    p.points.push_back(Point(ax[1], y + m.y_short + m.hex_side + m.y_short - m.y_offset));
                    p.points.push_back(Point(ax[1], y + y_step + m.y_offset));
                }
                ax[0] = ax[0] + m.distance;
                ax[1] = ax[1] + m.distance;
                std::swap(ax[0], ax[1]); // draw symmetrical pattern
                x += m.distance;
            }
            p.rotate(-direction.first, pivot);
            all_polylines.push_back(p);
        }
    }
    
    all_polylines = intersection_pl(std::move(all_polylines), expolygon);
    if (params.dont_connect() || all_polylines.size() <= 1)
        append(polylines_out, chain_polylines(std::move(all_polylines)));
    else
        connect_infill(std::move(all_polylines), expolygon, polylines_out, this->spacing, params);
}

} // namespace Slic3r
