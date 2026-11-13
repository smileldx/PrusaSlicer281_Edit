///|/ Copyright (c) Prusa Research 2016 - 2020 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/Fill/Honeycomb.pm:
///|/ Copyright (c) Prusa Research 2016 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2012 - 2015 Alessandro Ranellucci @alranel
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_FillHoneycomb_hpp_
#define slic3r_FillHoneycomb_hpp_

#include <math.h>
#include <stddef.h>
#include <map>
#include <utility>
#include <cmath>
#include <cstddef>

#include "libslic3r/libslic3r.h"
#include "FillBase.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"

namespace Slic3r {

class FillHoneycomb : public Fill
{
public:
    // ldx[修改蜂窝换向填充] Toggle between same-direction alignment (true) and alternating 60-degree interlocking honeycomb infill (false).
    static constexpr bool SameDirectionHoneycomb = false;

    ~FillHoneycomb() override {}
    bool is_self_crossing() override { return false; }

    // ldx[修改蜂窝换向填充] Apply per-layer 0/60/120 deg rotation when SameDirectionHoneycomb is false, supporting single-angle fill_angles input.
    std::pair<float, Point> _infill_direction(const Surface *surface) const override;

protected:
    Fill* clone() const override { return new FillHoneycomb(*this); };
	void _fill_surface_single(
	    const FillParams                &params, 
	    unsigned int                     thickness_layers,
	    const std::pair<float, Point>   &direction, 
	    ExPolygon                 		 expolygon,
	    Polylines                       &polylines_out) override;

	// Caching the 
	struct CacheID 
	{
		//ldx[line_spacing] Include resolved distance, min_spacing and custom mode flag in CacheID so mode/line_spacing/adjust changes don't reuse stale geometry.
		CacheID(coord_t adistance, coord_t amin_spacing, bool acustom) :
			distance(adistance), min_spacing(amin_spacing), use_custom_spacing(acustom) {}
		coord_t		distance;
		coord_t		min_spacing;
		bool		use_custom_spacing;
		bool operator<(const CacheID &other) const
		{
			if (distance != other.distance)
				return distance < other.distance;
			if (min_spacing != other.min_spacing)
				return min_spacing < other.min_spacing;
			return int(use_custom_spacing) < int(other.use_custom_spacing);
		}
		bool operator==(const CacheID &other) const
			{ return distance == other.distance && min_spacing == other.min_spacing && use_custom_spacing == other.use_custom_spacing; }
	};
	struct CacheData
	{
		coord_t	distance;
        coord_t hex_side;
        coord_t hex_width;
        coord_t	pattern_height;
        coord_t y_short;
        coord_t x_offset;
        coord_t	y_offset;
        Point	hex_center;
    };
    typedef std::map<CacheID, CacheData> Cache;
	Cache cache;

    float _layer_angle(size_t idx) const override { return float(M_PI/3.) * (idx % 3); }
};

} // namespace Slic3r

#endif // slic3r_FillHoneycomb_hpp_
